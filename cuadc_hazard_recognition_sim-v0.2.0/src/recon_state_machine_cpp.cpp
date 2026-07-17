#include <rclcpp/rclcpp.hpp>

#include <cuadc_hazard_recognition_sim/msg/hazard_detection.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/command_tol.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <fstream>
#include <future>
#include <iomanip>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

using namespace std::chrono_literals;
using HazardDetection = cuadc_hazard_recognition_sim::msg::HazardDetection;

enum class MissionState {
  WAITING_FCU, SETTING_GUIDED, ARMING, TAKEOFF, SCANNING, RETURN_HOME, LANDING, DONE
};

struct Point3 {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

struct SearchArea {
  Point3 center_world;
  Point3 center_rtk_enu;
  double size_x = 0.0;
  double size_y = 0.0;
  double world_to_rtk_yaw = 0.0;
  bool valid = false;
};

struct CoverageWaypoint {
  Point3 point;
  std::size_t lane_index = 0;
};

struct Observation {
  std::string predicted;
  double confidence = 0.0;
  std::size_t merged_views = 1;
  Point3 local_position;
  Point3 rtk_enu;
  bool has_rtk = false;
  std::size_t route_index = 0;
};

struct Segment {
  Point3 start;
  Point3 end;
  double duration_s = 1.0;
  rclcpp::Time start_time;
};

class ReconStateMachineCpp : public rclcpp::Node {
public:
  ReconStateMachineCpp() : Node("recon_state_machine_cpp") {
    declare_parameter<double>("takeoff_alt", 1.5);
    declare_parameter<double>("waypoint_accept_radius", 0.30);
    declare_parameter<double>("takeoff_timeout_s", 60.0);
    declare_parameter<double>("mission_timeout_s", 300.0);
    declare_parameter<double>("trajectory_speed", 0.65);
    declare_parameter<double>("min_segment_time_s", 1.5);
    declare_parameter<double>("coverage_lane_spacing_m", 0.70);
    declare_parameter<double>("coverage_edge_inset_x_m", 0.40);
    declare_parameter<double>("coverage_edge_inset_y_m", 0.65);
    declare_parameter<double>("detection_pause_s", 3.0);
    declare_parameter<double>("detection_centering_timeout_s", 5.0);
    declare_parameter<double>("visual_min_confidence", 0.25);
    declare_parameter<int>("visual_min_consecutive", 3);
    declare_parameter<double>("visual_weak_confidence", 0.05);
    declare_parameter<int>("visual_weak_min_consecutive", 8);
    declare_parameter<double>("visual_center_gate_fraction", 0.90);
    declare_parameter<double>("observation_merge_radius_m", 0.65);
    declare_parameter<double>("observation_same_class_merge_radius_m", 0.85);
    declare_parameter<int>("camera_width", 848);
    declare_parameter<int>("camera_height", 480);
    declare_parameter<double>("camera_footprint_x_m", 1.02);
    declare_parameter<double>("camera_footprint_y_m", 1.80);
    declare_parameter<double>("camera_image_yaw_deg", 0.0);
    declare_parameter<std::string>("generated_scene_path", "");
    declare_parameter<std::string>("coordinate_source", "auto_rtk");
    declare_parameter<double>("rtk_fix_timeout_s", 8.0);
    declare_parameter<double>("rtk_to_mavros_yaw_deg", 0.0);
    declare_parameter<std::string>(
      "result_path", "/tmp/cuadc_hazard_search_observations.json");

    const auto gp = [this](const char * name) {return get_parameter(name);};
    takeoff_alt_ = gp("takeoff_alt").as_double();
    accept_radius_ = gp("waypoint_accept_radius").as_double();
    takeoff_timeout_s_ = gp("takeoff_timeout_s").as_double();
    mission_timeout_s_ = gp("mission_timeout_s").as_double();
    trajectory_speed_ = std::max(0.1, gp("trajectory_speed").as_double());
    min_segment_time_s_ = std::max(0.5, gp("min_segment_time_s").as_double());
    lane_spacing_ = std::max(0.1, gp("coverage_lane_spacing_m").as_double());
    edge_inset_x_ = std::max(0.0, gp("coverage_edge_inset_x_m").as_double());
    edge_inset_y_ = std::max(0.0, gp("coverage_edge_inset_y_m").as_double());
    detection_pause_s_ = std::max(0.5, gp("detection_pause_s").as_double());
    detection_centering_timeout_s_ = std::max(
      1.0, gp("detection_centering_timeout_s").as_double());
    visual_min_confidence_ = std::clamp(
      gp("visual_min_confidence").as_double(), 0.0, 1.0);
    visual_min_consecutive_ = std::max(
      1, static_cast<int>(gp("visual_min_consecutive").as_int()));
    visual_weak_confidence_ = std::clamp(
      gp("visual_weak_confidence").as_double(), 0.0, visual_min_confidence_);
    visual_weak_min_consecutive_ = std::max(
      visual_min_consecutive_ + 1,
      static_cast<int>(gp("visual_weak_min_consecutive").as_int()));
    visual_center_gate_fraction_ = std::clamp(
      gp("visual_center_gate_fraction").as_double(), 0.05, 1.0);
    observation_merge_radius_ = std::max(
      0.1, gp("observation_merge_radius_m").as_double());
    observation_same_class_merge_radius_ = std::max(
      observation_merge_radius_,
      gp("observation_same_class_merge_radius_m").as_double());
    camera_width_ = std::max(1, static_cast<int>(gp("camera_width").as_int()));
    camera_height_ = std::max(1, static_cast<int>(gp("camera_height").as_int()));
    camera_footprint_x_ = std::max(0.1, gp("camera_footprint_x_m").as_double());
    camera_footprint_y_ = std::max(0.1, gp("camera_footprint_y_m").as_double());
    camera_image_yaw_ = gp("camera_image_yaw_deg").as_double() * M_PI / 180.0;
    generated_scene_path_ = gp("generated_scene_path").as_string();
    coordinate_source_ = gp("coordinate_source").as_string();
    rtk_fix_timeout_s_ = std::max(0.0, gp("rtk_fix_timeout_s").as_double());
    rtk_to_local_yaw_ = gp("rtk_to_mavros_yaw_deg").as_double() * M_PI / 180.0;
    result_path_ = gp("result_path").as_string();

    load_public_scene_metadata();
    if (!search_area_.valid) {
      throw std::runtime_error(
              "recognition_area metadata is required; bucket positions are intentionally not accepted");
    }

    const auto state_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local();
    const auto sensor_qos = rclcpp::SensorDataQoS();
    state_sub_ = create_subscription<mavros_msgs::msg::State>(
      "/mavros/state", state_qos,
      [this](const mavros_msgs::msg::State::SharedPtr msg) {current_state_ = *msg;});
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/mavros/local_position/odom", sensor_qos,
      std::bind(&ReconStateMachineCpp::odom_cb, this, std::placeholders::_1));
    fix_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
      "/mavros/global_position/global", sensor_qos,
      std::bind(&ReconStateMachineCpp::fix_cb, this, std::placeholders::_1));
    detection_sub_ = create_subscription<HazardDetection>(
      "/perception/hazard_detection", 10,
      std::bind(&ReconStateMachineCpp::detection_cb, this, std::placeholders::_1));
    setpoint_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "/mavros/setpoint_position/local", rclcpp::QoS(10).reliable());
    arming_cli_ = create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
    set_mode_cli_ = create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");
    takeoff_cli_ = create_client<mavros_msgs::srv::CommandTOL>("/mavros/cmd/takeoff");
    land_cli_ = create_client<mavros_msgs::srv::CommandTOL>("/mavros/cmd/land");

    mission_start_time_ = now();
    state_enter_time_ = now();
    last_state_log_ = now();
    timer_ = create_wall_timer(50ms, std::bind(&ReconStateMachineCpp::control_loop, this));
    RCLCPP_INFO(
      get_logger(),
      "Unknown-target coverage state machine: area=%.2fx%.2f m coordinates=%s; "
      "random bucket positions are not loaded",
      search_area_.size_x, search_area_.size_y, coordinate_source_.c_str());
  }

private:
  void odom_cb(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_position_ = Point3{
      msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z};
    have_local_position_ = true;
    if (!home_.has_value() && mission_state_ == MissionState::WAITING_FCU) {
      home_ = current_position_;
      RCLCPP_INFO(
        get_logger(), "MAVROS local origin locked at (%.3f, %.3f, %.3f)",
        home_->x, home_->y, home_->z);
    }
    try_lock_rtk();
  }

  void fix_cb(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
    if (msg->status.status < sensor_msgs::msg::NavSatStatus::STATUS_FIX ||
      !std::isfinite(msg->latitude) || !std::isfinite(msg->longitude) ||
      !std::isfinite(msg->altitude) ||
      std::abs(msg->latitude) > 90.0 || std::abs(msg->longitude) > 180.0)
    {
      return;
    }
    latest_fix_ = *msg;
    have_valid_fix_ = true;
    try_lock_rtk();
  }

  void try_lock_rtk() {
    if (rtk_locked_ || !have_valid_fix_ || !have_local_position_ || !have_rtk_reference_) {
      return;
    }
    rtk_rover_anchor_ = geodetic_to_rtk_enu(
      latest_fix_.latitude, latest_fix_.longitude, latest_fix_.altitude);
    mavros_local_anchor_ = current_position_;
    rtk_locked_ = true;
    using_spawn_fallback_ = false;
    RCLCPP_INFO(
      get_logger(),
      "RTK horizontal transform locked: rover EN=(%.3f, %.3f), local EN=(%.3f, %.3f), "
      "yaw=%.2f deg; RTK up diagnostic=%.3f m is not used for flight altitude",
      rtk_rover_anchor_.x, rtk_rover_anchor_.y,
      mavros_local_anchor_.x, mavros_local_anchor_.y,
      rtk_to_local_yaw_ * 180.0 / M_PI, rtk_rover_anchor_.z);
  }

  void reset_candidate() {
    candidate_class_scores_.clear();
    candidate_class_peaks_.clear();
    candidate_frame_count_ = 0;
    candidate_position_samples_ = 0;
    candidate_position_ = Point3{};
  }

  Point3 estimate_object_local(const HazardDetection & msg) const {
    const double image_dx =
      -(static_cast<double>(msg.center_v) - camera_height_ * 0.5) /
      static_cast<double>(camera_height_) * camera_footprint_x_;
    const double image_dy =
      -(static_cast<double>(msg.center_u) - camera_width_ * 0.5) /
      static_cast<double>(camera_width_) * camera_footprint_y_;
    const double c = std::cos(camera_image_yaw_);
    const double s = std::sin(camera_image_yaw_);
    return Point3{
      current_position_.x + c * image_dx - s * image_dy,
      current_position_.y + s * image_dx + c * image_dy,
      current_position_.z};
  }

  bool near_recorded_observation(
    const Point3 & position, const std::string & predicted) const
  {
    return std::any_of(
      observations_.begin(), observations_.end(),
      [&](const Observation & observation) {
        const double distance = distance_xy(position, observation.local_position);
        return distance <= observation_merge_radius_ ||
               (predicted == observation.predicted &&
               distance <= observation_same_class_merge_radius_);
      });
  }

  void record_observation(
    const std::string & predicted, double confidence, const Point3 & local_position)
  {
    auto existing = std::find_if(
      observations_.begin(), observations_.end(),
      [&](const Observation & observation) {
        const double distance = distance_xy(local_position, observation.local_position);
        return distance <= observation_merge_radius_ ||
               (predicted == observation.predicted &&
               distance <= observation_same_class_merge_radius_);
      });
    if (existing != observations_.end()) {
      const double old_views = static_cast<double>(existing->merged_views);
      const double new_views = old_views + 1.0;
      existing->local_position.x =
        (existing->local_position.x * old_views + local_position.x) / new_views;
      existing->local_position.y =
        (existing->local_position.y * old_views + local_position.y) / new_views;
      existing->local_position.z = local_position.z;
      ++existing->merged_views;
      if (confidence > existing->confidence) {
        existing->predicted = predicted;
        existing->confidence = confidence;
      }
      existing->route_index = scan_index_;
      if (rtk_locked_) {
        existing->rtk_enu = local_to_rtk_enu(existing->local_position);
        existing->has_rtk = true;
      }
      RCLCPP_INFO(
        get_logger(),
        "Merged repeated spatial observation: class=%s confidence=%.3f views=%zu",
        existing->predicted.c_str(), existing->confidence, existing->merged_views);
      return;
    }

    Observation observation;
    observation.predicted = predicted;
    observation.confidence = confidence;
    observation.local_position = local_position;
    observation.route_index = scan_index_;
    if (rtk_locked_) {
      observation.rtk_enu = local_to_rtk_enu(local_position);
      observation.has_rtk = true;
    }
    observations_.push_back(observation);
    RCLCPP_INFO(
      get_logger(),
      "Recorded unknown-target observation %zu: class=%s confidence=%.3f "
      "local=(%.2f,%.2f) route=%zu/%zu",
      observations_.size(), predicted.c_str(), confidence,
      local_position.x, local_position.y, scan_index_ + 1, scan_route_.size());
  }

  void detection_cb(const HazardDetection::SharedPtr msg) {
    if (mission_state_ != MissionState::SCANNING || !coverage_detection_enabled_ ||
      scan_index_ >= scan_route_.size())
    {
      return;
    }
    if (msg->class_name.empty() || msg->confidence < visual_weak_confidence_) {
      return;
    }
    const double du =
      std::abs(msg->center_u - camera_width_ * 0.5) / (camera_width_ * 0.5);
    const double dv =
      std::abs(msg->center_v - camera_height_ * 0.5) / (camera_height_ * 0.5);
    if (du > visual_center_gate_fraction_ || dv > visual_center_gate_fraction_) {
      return;
    }

    const Point3 estimated_position = estimate_object_local(*msg);
    if (!detection_hold_active_ &&
      near_recorded_observation(estimated_position, msg->class_name))
    {
      return;
    }

    if (!detection_hold_active_) {
      reset_candidate();
      detection_hold_active_ = true;
      detection_evidence_active_ = false;
      detection_hold_position_ = Point3{
        estimated_position.x, estimated_position.y,
        home_.value_or(Point3{}).z + takeoff_alt_};
      detection_centering_start_time_ = now();
      RCLCPP_INFO(
        get_logger(),
        "Potential target seen during coverage; visually centering on projected "
        "local position (%.2f,%.2f) before evidence accumulation",
        detection_hold_position_.x, detection_hold_position_.y);
      return;
    }
    if (!detection_evidence_active_) {
      if (distance_xy(estimated_position, detection_hold_position_) <=
        observation_same_class_merge_radius_)
      {
        detection_hold_position_.x =
          0.8 * detection_hold_position_.x + 0.2 * estimated_position.x;
        detection_hold_position_.y =
          0.8 * detection_hold_position_.y + 0.2 * estimated_position.y;
      }
      return;
    }
    if (candidate_position_samples_ > 0 &&
      distance_xy(estimated_position, candidate_position_) > observation_merge_radius_)
    {
      return;
    }

    ++candidate_frame_count_;
    ++candidate_position_samples_;
    const double samples = static_cast<double>(candidate_position_samples_);
    candidate_position_.x =
      candidate_position_.x * ((samples - 1.0) / samples) +
      estimated_position.x / samples;
    candidate_position_.y =
      candidate_position_.y * ((samples - 1.0) / samples) +
      estimated_position.y / samples;
    candidate_position_.z = estimated_position.z;

    if (!msg->class_names.empty() &&
      msg->class_names.size() == msg->class_confidences.size())
    {
      for (std::size_t i = 0; i < msg->class_names.size(); ++i) {
        const double score =
          std::clamp(static_cast<double>(msg->class_confidences[i]), 0.0, 1.0);
        candidate_class_scores_[msg->class_names[i]] += score;
        candidate_class_peaks_[msg->class_names[i]] =
          std::max(candidate_class_peaks_[msg->class_names[i]], score);
      }
    } else {
      const double score = std::clamp(static_cast<double>(msg->confidence), 0.0, 1.0);
      candidate_class_scores_[msg->class_name] += score;
      candidate_class_peaks_[msg->class_name] =
        std::max(candidate_class_peaks_[msg->class_name], score);
    }
  }

  bool finalize_detection_candidate() {
    if (candidate_frame_count_ <= 0 || candidate_class_scores_.empty()) {
      return false;
    }
    const auto best = std::max_element(
      candidate_class_scores_.begin(), candidate_class_scores_.end(),
      [](const auto & a, const auto & b) {return a.second < b.second;});
    const double average_score =
      best->second / static_cast<double>(candidate_frame_count_);
    const bool high_confirmation =
      candidate_frame_count_ >= visual_min_consecutive_ &&
      average_score >= visual_min_confidence_;
    const bool weak_confirmation =
      candidate_frame_count_ >= visual_weak_min_consecutive_ &&
      average_score >= visual_weak_confidence_;
    if (!high_confirmation && !weak_confirmation) {
      RCLCPP_INFO(
        get_logger(),
        "Detection evidence rejected: best=%s average=%.3f frames=%d",
        best->first.c_str(), average_score, candidate_frame_count_);
      return false;
    }

    const double peak = candidate_class_peaks_[best->first];
    RCLCPP_INFO(
      get_logger(),
      "Coverage visual confirmation: class=%s average=%.3f peak=%.3f "
      "frames=%d mode=%s",
      best->first.c_str(), average_score, peak, candidate_frame_count_,
      high_confirmation ? "high-window" : "weak-window");
    record_observation(best->first, average_score, candidate_position_);
    return true;
  }

  void control_loop() {
    check_service_results();
    if (publish_setpoints_) publish_setpoint(target_);
    if (seconds_since(last_state_log_) > 4.0) {
      last_state_log_ = now();
      RCLCPP_INFO(
        get_logger(),
        "state=%s fcu=%s pos=(%.2f,%.2f,%.2f) target=(%.2f,%.2f,%.2f) "
        "localization=%s coverage=%zu/%zu detections=%zu",
        state_name(mission_state_).c_str(),
        current_state_.connected ? "connected" : "disconnected",
        current_position_.x, current_position_.y, current_position_.z,
        target_.x, target_.y, target_.z,
        rtk_locked_ ? "RTK" : (using_spawn_fallback_ ? "SPAWN_FALLBACK" : "WAITING_RTK"),
        scan_index_, scan_route_.size(), observations_.size());
    }

    if (mission_started_ && seconds_since(mission_start_time_) > mission_timeout_s_ &&
      mission_state_ != MissionState::RETURN_HOME &&
      mission_state_ != MissionState::LANDING && mission_state_ != MissionState::DONE)
    {
      RCLCPP_WARN(get_logger(), "Coverage mission timeout; returning home");
      start_return_home();
    }

    switch (mission_state_) {
      case MissionState::WAITING_FCU:
        update_localization_fallback();
        if (current_state_.connected && home_.has_value() && localization_ready()) {
          enter_state(MissionState::SETTING_GUIDED);
        }
        break;
      case MissionState::SETTING_GUIDED:
        if (current_state_.mode == "GUIDED") {
          enter_state(MissionState::ARMING);
        } else if (!pending_set_mode_future_.valid()) {
          call_set_mode("GUIDED");
        }
        break;
      case MissionState::ARMING:
        if (current_state_.armed) {
          enter_state(MissionState::TAKEOFF);
        } else if (!pending_arm_future_.valid()) {
          call_arm(true);
        }
        break;
      case MissionState::TAKEOFF:
        publish_setpoints_ = false;
        if (!takeoff_sent_) {
          call_takeoff(takeoff_alt_);
          takeoff_sent_ = true;
          state_enter_time_ = now();
        }
        if (relative_altitude() >= takeoff_alt_ * 0.90 ||
          seconds_since(state_enter_time_) > takeoff_timeout_s_)
        {
          publish_setpoints_ = true;
          start_coverage_scan();
        }
        break;
      case MissionState::SCANNING:
        update_coverage_trajectory();
        break;
      case MissionState::RETURN_HOME:
        update_return_trajectory();
        break;
      case MissionState::LANDING:
        publish_setpoints_ = false;
        if (!land_sent_) {
          call_land();
          land_sent_ = true;
        }
        if (relative_altitude() < 0.20) {
          enter_state(MissionState::DONE);
        }
        break;
      case MissionState::DONE:
        if (!result_written_) {
          write_observations();
          result_written_ = true;
          RCLCPP_INFO(
            get_logger(), "Coverage mission complete; raw observations=%s",
            result_path_.c_str());
          rclcpp::shutdown();
        }
        break;
    }
  }

  void update_localization_fallback() {
    if (rtk_locked_ || using_spawn_fallback_ || coordinate_source_ == "rtk") {
      return;
    }
    if (!current_state_.connected || !have_local_position_) {
      localization_wait_start_.reset();
      return;
    }
    if (!localization_wait_start_.has_value()) {
      localization_wait_start_ = now();
      return;
    }
    if (seconds_since(*localization_wait_start_) >= rtk_fix_timeout_s_) {
      using_spawn_fallback_ = true;
      RCLCPP_WARN(
        get_logger(),
        "No valid RTK fix %.1f s after FCU/local-pose readiness; using explicit spawn-relative fallback",
        rtk_fix_timeout_s_);
    }
  }

  bool localization_ready() const {
    if (coordinate_source_ == "spawn_relative") {
      return true;
    }
    if (coordinate_source_ == "rtk") {
      return rtk_locked_;
    }
    return rtk_locked_ || using_spawn_fallback_;
  }

  void load_public_scene_metadata() {
    if (generated_scene_path_.empty()) {
      return;
    }
    try {
      const YAML::Node scene = YAML::LoadFile(generated_scene_path_);
      scene_seed_ = scene["seed"] ? scene["seed"].as<int>() : 0;
      const auto rtk = scene["rtk_base"];
      if (rtk) {
        rtk_base_lat_ = rtk["latitude_deg"].as<double>();
        rtk_base_lon_ = rtk["longitude_deg"].as<double>();
        rtk_base_alt_ = rtk["altitude_m"].as<double>();
        have_rtk_reference_ = true;
      }
      const auto vehicle = scene["vehicle"];
      if (vehicle && vehicle["pose"] && vehicle["pose"].size() >= 2) {
        vehicle_spawn_world_ = Point3{
          vehicle["pose"][0].as<double>(), vehicle["pose"][1].as<double>(), 0.0};
      }
      const auto area = scene["recognition_area"];
      if (area && area["center_world"] && area["size"] &&
        area["center_rtk_enu"] && area["center_world"].size() >= 2 &&
        area["size"].size() >= 2 && area["center_rtk_enu"].size() >= 2)
      {
        search_area_.center_world = Point3{
          area["center_world"][0].as<double>(), area["center_world"][1].as<double>(), 0.0};
        search_area_.center_rtk_enu = Point3{
          area["center_rtk_enu"][0].as<double>(),
          area["center_rtk_enu"][1].as<double>(), 0.0};
        search_area_.size_x = area["size"][0].as<double>();
        search_area_.size_y = area["size"][1].as<double>();
        search_area_.world_to_rtk_yaw =
          area["world_to_rtk_yaw_deg"].as<double>() * M_PI / 180.0;
        search_area_.valid = search_area_.size_x > 0.0 && search_area_.size_y > 0.0;
      }
      // Deliberately do not read scene["recon_targets"]. Those positions are
      // randomized ground truth and are reserved for post-mission evaluation.
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(
        get_logger(), "Cannot load public scene metadata %s: %s",
        generated_scene_path_.c_str(), ex.what());
    }
  }

  Point3 geodetic_to_rtk_enu(double lat, double lon, double alt) const {
    constexpr double semi_major = 6378137.0;
    constexpr double flattening = 1.0 / 298.257223563;
    constexpr double eccentricity_sq = flattening * (2.0 - flattening);
    const auto to_ecef = [](double latitude_deg, double longitude_deg, double height) {
      const double latitude = latitude_deg * M_PI / 180.0;
      const double longitude = longitude_deg * M_PI / 180.0;
      const double sin_lat = std::sin(latitude);
      const double prime_vertical =
        semi_major / std::sqrt(1.0 - eccentricity_sq * sin_lat * sin_lat);
      return Point3{
        (prime_vertical + height) * std::cos(latitude) * std::cos(longitude),
        (prime_vertical + height) * std::cos(latitude) * std::sin(longitude),
        (prime_vertical * (1.0 - eccentricity_sq) + height) * sin_lat};
    };
    const Point3 base = to_ecef(rtk_base_lat_, rtk_base_lon_, rtk_base_alt_);
    const Point3 rover = to_ecef(lat, lon, alt);
    const double dx = rover.x - base.x;
    const double dy = rover.y - base.y;
    const double dz = rover.z - base.z;
    const double lat0 = rtk_base_lat_ * M_PI / 180.0;
    const double lon0 = rtk_base_lon_ * M_PI / 180.0;
    return Point3{
      -std::sin(lon0) * dx + std::cos(lon0) * dy,
      -std::sin(lat0) * std::cos(lon0) * dx -
        std::sin(lat0) * std::sin(lon0) * dy + std::cos(lat0) * dz,
      std::cos(lat0) * std::cos(lon0) * dx +
        std::cos(lat0) * std::sin(lon0) * dy + std::sin(lat0) * dz};
  }

  Point3 rtk_to_local(const Point3 & rtk) const {
    const double east = rtk.x - rtk_rover_anchor_.x;
    const double north = rtk.y - rtk_rover_anchor_.y;
    return Point3{
      mavros_local_anchor_.x +
        std::cos(rtk_to_local_yaw_) * east - std::sin(rtk_to_local_yaw_) * north,
      mavros_local_anchor_.y +
        std::sin(rtk_to_local_yaw_) * east + std::cos(rtk_to_local_yaw_) * north,
      home_.value_or(Point3{}).z + takeoff_alt_};
  }

  Point3 local_to_rtk_enu(const Point3 & local) const {
    const double dx = local.x - mavros_local_anchor_.x;
    const double dy = local.y - mavros_local_anchor_.y;
    return Point3{
      rtk_rover_anchor_.x +
        std::cos(rtk_to_local_yaw_) * dx + std::sin(rtk_to_local_yaw_) * dy,
      rtk_rover_anchor_.y -
        std::sin(rtk_to_local_yaw_) * dx + std::cos(rtk_to_local_yaw_) * dy,
      0.0};
  }

  Point3 area_offset_to_local(double offset_x, double offset_y) const {
    if (rtk_locked_ && coordinate_source_ != "spawn_relative") {
      const double c = std::cos(search_area_.world_to_rtk_yaw);
      const double s = std::sin(search_area_.world_to_rtk_yaw);
      const Point3 rtk{
        search_area_.center_rtk_enu.x + c * offset_x - s * offset_y,
        search_area_.center_rtk_enu.y + s * offset_x + c * offset_y,
        0.0};
      return rtk_to_local(rtk);
    }
    const auto home = home_.value_or(Point3{});
    return Point3{
      home.x + search_area_.center_world.x + offset_x - vehicle_spawn_world_.x,
      home.y + search_area_.center_world.y + offset_y - vehicle_spawn_world_.y,
      home.z + takeoff_alt_};
  }

  void start_coverage_scan() {
    scan_route_.clear();
    const double half_x = search_area_.size_x * 0.5;
    const double half_y = search_area_.size_y * 0.5;
    const double inset_x = std::min(edge_inset_x_, half_x);
    const double inset_y = std::min(edge_inset_y_, half_y);
    const double x_min = -half_x + inset_x;
    const double x_max = half_x - inset_x;
    const double y_min = -half_y + inset_y;
    const double y_max = half_y - inset_y;
    const double x_span = std::max(0.0, x_max - x_min);
    const std::size_t lane_count =
      x_span < 1e-6 ? 1U :
      static_cast<std::size_t>(std::ceil(x_span / lane_spacing_ - 1e-9)) + 1U;

    std::vector<double> lane_offsets(lane_count, 0.0);
    for (std::size_t i = 0; i < lane_count; ++i) {
      lane_offsets[i] =
        lane_count == 1 ? 0.0 :
        x_min + x_span * static_cast<double>(i) / static_cast<double>(lane_count - 1);
    }

    bool reverse_lanes = false;
    bool first_from_high_y = false;
    double best_entry_distance = std::numeric_limits<double>::infinity();
    const Point3 entry_reference = current_at_altitude();
    for (const bool reverse : {false, true}) {
      for (const bool high_y : {false, true}) {
        const double first_x = reverse ? lane_offsets.back() : lane_offsets.front();
        const Point3 candidate =
          area_offset_to_local(first_x, high_y ? y_max : y_min);
        const double distance = distance_xy(entry_reference, candidate);
        if (distance < best_entry_distance) {
          best_entry_distance = distance;
          reverse_lanes = reverse;
          first_from_high_y = high_y;
        }
      }
    }

    for (std::size_t order = 0; order < lane_count; ++order) {
      const std::size_t lane = reverse_lanes ? lane_count - 1U - order : order;
      const bool start_high = first_from_high_y ^ (order % 2U == 1U);
      const double first_y = start_high ? y_max : y_min;
      const double second_y = start_high ? y_min : y_max;
      scan_route_.push_back(
        CoverageWaypoint{area_offset_to_local(lane_offsets[lane], first_y), lane});
      if (std::abs(second_y - first_y) > 1e-6) {
        scan_route_.push_back(
          CoverageWaypoint{area_offset_to_local(lane_offsets[lane], second_y), lane});
      }
    }

    scan_index_ = 0;
    detection_hold_active_ = false;
    detection_evidence_active_ = false;
    coverage_detection_enabled_ = false;
    reset_candidate();
    mission_start_time_ = now();
    mission_started_ = true;
    if (scan_route_.empty()) {
      start_return_home();
      return;
    }

    RCLCPP_INFO(
      get_logger(),
      "Coverage route planned from public area only: lanes=%zu waypoints=%zu "
      "spacing=%.2f m footprint=%.2fx%.2f m; random bucket positions remain unknown",
      lane_count, scan_route_.size(), lane_spacing_,
      camera_footprint_x_, camera_footprint_y_);
    for (std::size_t i = 0; i < scan_route_.size(); ++i) {
      RCLCPP_INFO(
        get_logger(), "Coverage waypoint %zu/%zu lane=%zu local ENU=(%.2f,%.2f,%.2f)",
        i + 1, scan_route_.size(), scan_route_[i].lane_index + 1,
        scan_route_[i].point.x, scan_route_[i].point.y, scan_route_[i].point.z);
    }
    start_segment(current_at_altitude(), scan_route_.front().point);
    enter_state(MissionState::SCANNING);
  }

  void update_coverage_trajectory() {
    if (scan_index_ >= scan_route_.size()) {
      start_return_home();
      return;
    }

    if (detection_hold_active_) {
      target_ = detection_hold_position_;
      if (!detection_evidence_active_) {
        if (distance_xyz(current_position_, detection_hold_position_) <= accept_radius_) {
          detection_evidence_active_ = true;
          detection_hold_start_time_ = now();
          reset_candidate();
          RCLCPP_INFO(
            get_logger(),
            "Visual target centered; accumulating full class evidence for %.1f s",
            detection_pause_s_);
        } else if (seconds_since(detection_centering_start_time_) >=
          detection_centering_timeout_s_)
        {
          RCLCPP_INFO(
            get_logger(), "Visual centering timed out; resuming coverage route");
          detection_hold_active_ = false;
          reset_candidate();
          start_segment(current_at_altitude(), scan_route_[scan_index_].point);
        }
        return;
      }
      if (seconds_since(detection_hold_start_time_) >= detection_pause_s_) {
        const bool confirmed = finalize_detection_candidate();
        if (!confirmed) {
          RCLCPP_INFO(
            get_logger(),
            "Detection pause expired without sufficient fused evidence; resuming coverage");
        }
        detection_hold_active_ = false;
        detection_evidence_active_ = false;
        reset_candidate();
        start_segment(current_at_altitude(), scan_route_[scan_index_].point);
      }
      return;
    }

    target_ = sample_segment();
    update_yaw_to_target(target_);
    if (!segment_finished()) {
      return;
    }
    target_ = active_segment_.end;
    if (distance_xyz(current_position_, active_segment_.end) > accept_radius_) {
      return;
    }

    RCLCPP_INFO(
      get_logger(), "Coverage waypoint reached %zu/%zu lane=%zu",
      scan_index_ + 1, scan_route_.size(), scan_route_[scan_index_].lane_index + 1);
    if (!coverage_detection_enabled_) {
      coverage_detection_enabled_ = true;
      RCLCPP_INFO(
        get_logger(),
        "Recognition gate enabled at the first public-area coverage waypoint");
    }
    ++scan_index_;
    if (scan_index_ >= scan_route_.size()) {
      RCLCPP_INFO(
        get_logger(),
        "Entire specified recognition area covered; observations=%zu",
        observations_.size());
      start_return_home();
      return;
    }
    start_segment(current_at_altitude(), scan_route_[scan_index_].point);
  }

  void start_return_home() {
    const auto home = home_.value_or(Point3{});
    start_segment(
      current_at_altitude(), Point3{home.x, home.y, home.z + takeoff_alt_});
    detection_hold_active_ = false;
    enter_state(MissionState::RETURN_HOME);
  }

  void update_return_trajectory() {
    target_ = sample_segment();
    update_yaw_to_target(target_);
    if (segment_finished() &&
      distance_xyz(current_position_, active_segment_.end) <= accept_radius_)
    {
      enter_state(MissionState::LANDING);
    }
  }

  void start_segment(const Point3 & start, const Point3 & end) {
    const double duration =
      std::max(min_segment_time_s_, distance_xyz(start, end) / trajectory_speed_);
    active_segment_ = Segment{start, end, duration, now()};
    target_ = start;
    update_yaw_to_target(end);
  }

  Point3 sample_segment() const {
    const double elapsed =
      std::clamp(seconds_since(active_segment_.start_time), 0.0, active_segment_.duration_s);
    const double tau = elapsed / active_segment_.duration_s;
    const double t2 = tau * tau;
    const double t3 = t2 * tau;
    const double t4 = t3 * tau;
    const double t5 = t4 * tau;
    const double t6 = t5 * tau;
    const double t7 = t6 * tau;
    const double blend = 35.0 * t4 - 84.0 * t5 + 70.0 * t6 - 20.0 * t7;
    return Point3{
      active_segment_.start.x + (active_segment_.end.x - active_segment_.start.x) * blend,
      active_segment_.start.y + (active_segment_.end.y - active_segment_.start.y) * blend,
      active_segment_.start.z + (active_segment_.end.z - active_segment_.start.z) * blend};
  }

  bool segment_finished() const {
    return seconds_since(active_segment_.start_time) >= active_segment_.duration_s;
  }

  Point3 current_at_altitude() const {
    return Point3{
      current_position_.x, current_position_.y,
      home_.value_or(Point3{}).z + takeoff_alt_};
  }

  double relative_altitude() const {
    return current_position_.z - home_.value_or(Point3{}).z;
  }

  void publish_setpoint(const Point3 & p) {
    geometry_msgs::msg::PoseStamped msg;
    msg.header.stamp = now();
    msg.header.frame_id = "map";
    msg.pose.position.x = p.x;
    msg.pose.position.y = p.y;
    msg.pose.position.z = p.z;
    msg.pose.orientation.z = std::sin(current_yaw_ * 0.5);
    msg.pose.orientation.w = std::cos(current_yaw_ * 0.5);
    setpoint_pub_->publish(msg);
  }

  void update_yaw_to_target(const Point3 & p) {
    const double dx = p.x - current_position_.x;
    const double dy = p.y - current_position_.y;
    if (std::hypot(dx, dy) > 0.2) {
      current_yaw_ = std::atan2(dy, dx);
    }
  }

  void call_set_mode(const std::string & mode) {
    if (!set_mode_cli_->service_is_ready()) {
      return;
    }
    auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    request->custom_mode = mode;
    pending_set_mode_future_ = set_mode_cli_->async_send_request(request).future.share();
  }

  void call_arm(bool arm) {
    if (!arming_cli_->service_is_ready()) {
      return;
    }
    auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
    request->value = arm;
    pending_arm_future_ = arming_cli_->async_send_request(request).future.share();
  }

  void call_takeoff(double altitude) {
    if (!takeoff_cli_->service_is_ready()) {
      takeoff_sent_ = false;
      return;
    }
    auto request = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
    request->altitude = static_cast<float>(altitude);
    pending_takeoff_future_ = takeoff_cli_->async_send_request(request).future.share();
  }

  void call_land() {
    if (!land_cli_->service_is_ready()) {
      land_sent_ = false;
      return;
    }
    auto request = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
    land_cli_->async_send_request(request);
  }

  void check_service_results() {
    if (pending_set_mode_future_.valid() &&
      pending_set_mode_future_.wait_for(0s) == std::future_status::ready)
    {
      const auto result = pending_set_mode_future_.get();
      if (!result->mode_sent) {
        RCLCPP_WARN(get_logger(), "Set mode rejected; retrying");
      }
      pending_set_mode_future_ = {};
    }
    if (pending_arm_future_.valid() &&
      pending_arm_future_.wait_for(0s) == std::future_status::ready)
    {
      const auto result = pending_arm_future_.get();
      if (!result->success) {
        RCLCPP_WARN(get_logger(), "Arming rejected; retrying");
      }
      pending_arm_future_ = {};
    }
    if (pending_takeoff_future_.valid() &&
      pending_takeoff_future_.wait_for(0s) == std::future_status::ready)
    {
      const auto result = pending_takeoff_future_.get();
      if (!result->success) {
        RCLCPP_WARN(get_logger(), "Takeoff rejected; retrying");
        takeoff_sent_ = false;
      }
      pending_takeoff_future_ = {};
    }
  }

  void write_observations() {
    std::ofstream output(result_path_);
    if (!output) {
      RCLCPP_ERROR(get_logger(), "Cannot write %s", result_path_.c_str());
      return;
    }
    output << "{\n";
    output << "  \"seed\": " << scene_seed_ << ",\n";
    output << "  \"mission\": \"coverage_search_unknown_bucket_positions\",\n";
    output << "  \"state_machine_has_ground_truth\": false,\n";
    output << "  \"localization\": \""
           << (rtk_locked_ ? "rtk" : "spawn_fallback") << "\",\n";
    output << "  \"coverage_waypoints\": " << scan_route_.size() << ",\n";
    output << "  \"coverage_completed\": "
           << (scan_index_ >= scan_route_.size() ? "true" : "false") << ",\n";
    output << "  \"observations\": [\n";
    for (std::size_t i = 0; i < observations_.size(); ++i) {
      const auto & observation = observations_[i];
      output << "    {\"class\": \"" << observation.predicted
             << "\", \"confidence\": " << std::fixed << std::setprecision(4)
             << observation.confidence << ", \"position_local\": ["
             << observation.local_position.x << ", " << observation.local_position.y
             << "], \"route_index\": " << observation.route_index
             << ", \"merged_views\": " << observation.merged_views;
      if (observation.has_rtk) {
        output << ", \"position_rtk_enu\": ["
               << observation.rtk_enu.x << ", " << observation.rtk_enu.y << "]";
      }
      output << "}";
      if (i + 1 < observations_.size()) {
        output << ',';
      }
      output << '\n';
    }
    output << "  ],\n";
    output << "  \"detected\": " << observations_.size() << "\n";
    output << "}\n";
  }

  void enter_state(MissionState state) {
    if (mission_state_ == state) {
      return;
    }
    mission_state_ = state;
    state_enter_time_ = now();
    RCLCPP_INFO(get_logger(), "Enter %s", state_name(state).c_str());
  }

  double seconds_since(const rclcpp::Time & start) const {
    return (now() - start).seconds();
  }

  static double distance_xy(const Point3 & a, const Point3 & b) {
    return std::hypot(a.x - b.x, a.y - b.y);
  }

  static double distance_xyz(const Point3 & a, const Point3 & b) {
    return std::hypot(distance_xy(a, b), a.z - b.z);
  }

  static std::string state_name(MissionState state) {
    switch (state) {
      case MissionState::WAITING_FCU: return "WAITING_FCU";
      case MissionState::SETTING_GUIDED: return "SETTING_GUIDED";
      case MissionState::ARMING: return "ARMING";
      case MissionState::TAKEOFF: return "TAKEOFF";
      case MissionState::SCANNING: return "SCANNING";
      case MissionState::RETURN_HOME: return "RETURN_HOME";
      case MissionState::LANDING: return "LANDING";
      case MissionState::DONE: return "DONE";
    }
    return "UNKNOWN";
  }

  double takeoff_alt_ = 1.5;
  double accept_radius_ = 0.3;
  double takeoff_timeout_s_ = 60.0;
  double mission_timeout_s_ = 300.0;
  double trajectory_speed_ = 0.65;
  double min_segment_time_s_ = 1.5;
  double lane_spacing_ = 0.70;
  double edge_inset_x_ = 0.40;
  double edge_inset_y_ = 0.65;
  double detection_pause_s_ = 3.0;
  double detection_centering_timeout_s_ = 5.0;
  double visual_min_confidence_ = 0.25;
  double visual_weak_confidence_ = 0.05;
  double visual_center_gate_fraction_ = 0.90;
  double observation_merge_radius_ = 0.65;
  double observation_same_class_merge_radius_ = 0.85;
  double camera_footprint_x_ = 1.02;
  double camera_footprint_y_ = 1.80;
  double camera_image_yaw_ = 0.0;
  double rtk_fix_timeout_s_ = 8.0;
  double rtk_to_local_yaw_ = 0.0;
  int visual_min_consecutive_ = 3;
  int visual_weak_min_consecutive_ = 8;
  int camera_width_ = 848;
  int camera_height_ = 480;
  int scene_seed_ = 0;

  bool have_local_position_ = false;
  bool have_valid_fix_ = false;
  bool have_rtk_reference_ = false;
  bool rtk_locked_ = false;
  bool using_spawn_fallback_ = false;
  bool publish_setpoints_ = true;
  bool takeoff_sent_ = false;
  bool land_sent_ = false;
  bool result_written_ = false;
  bool mission_started_ = false;
  bool detection_hold_active_ = false;
  bool detection_evidence_active_ = false;
  bool coverage_detection_enabled_ = false;

  std::string generated_scene_path_;
  std::string coordinate_source_;
  std::string result_path_;
  std::map<std::string, double> candidate_class_scores_;
  std::map<std::string, double> candidate_class_peaks_;
  int candidate_frame_count_ = 0;
  int candidate_position_samples_ = 0;
  double current_yaw_ = 0.0;
  double rtk_base_lat_ = 0.0;
  double rtk_base_lon_ = 0.0;
  double rtk_base_alt_ = 0.0;

  mavros_msgs::msg::State current_state_;
  sensor_msgs::msg::NavSatFix latest_fix_;
  Point3 current_position_;
  Point3 target_;
  Point3 vehicle_spawn_world_;
  Point3 rtk_rover_anchor_;
  Point3 mavros_local_anchor_;
  Point3 detection_hold_position_;
  Point3 candidate_position_;
  SearchArea search_area_;
  std::optional<Point3> home_;
  std::optional<rclcpp::Time> localization_wait_start_;
  std::vector<CoverageWaypoint> scan_route_;
  std::vector<Observation> observations_;
  std::size_t scan_index_ = 0;
  Segment active_segment_;
  MissionState mission_state_ = MissionState::WAITING_FCU;

  rclcpp::Time mission_start_time_;
  rclcpp::Time state_enter_time_;
  rclcpp::Time last_state_log_;
  rclcpp::Time detection_hold_start_time_;
  rclcpp::Time detection_centering_start_time_;

  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr fix_sub_;
  rclcpp::Subscription<HazardDetection>::SharedPtr detection_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr setpoint_pub_;
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_cli_;
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_cli_;
  rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr takeoff_cli_;
  rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr land_cli_;
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture pending_set_mode_future_;
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture pending_arm_future_;
  rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedFuture pending_takeoff_future_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ReconStateMachineCpp>());
  rclcpp::shutdown();
  return 0;
}
