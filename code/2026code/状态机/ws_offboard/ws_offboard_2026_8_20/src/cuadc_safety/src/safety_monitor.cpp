#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <exception>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>

#include <cuadc_interfaces/msg/mission_status.hpp>
#include <cuadc_interfaces/msg/payload_status.hpp>
#include <cuadc_interfaces/msg/safety_status.hpp>
#include <cuadc_recon/evidence_recorder.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

class SafetyMonitor final : public rclcpp::Node
{
public:
  SafetyMonitor()
  : Node("safety_monitor"), started_(Clock::now())
  {
    declare_parameter<bool>("flight_enable", false);
    declare_parameter<std::string>("config_bundle_sha256", "");
    declare_parameter<std::string>("log_directory", "");
    declare_parameter<std::string>("recon_evidence_directory", "");
    declare_parameter<std::string>(
      "recon_image_topic", "/camera/camera/color/image_raw");
    declare_parameter<double>("recon_capture_interval_s", 1.0);
    declare_parameter<int64_t>("recon_jpeg_quality", 90);
    declare_parameter<double>("mavros_timeout_s", 1.5);
    declare_parameter<double>("odom_timeout_s", 0.6);
    declare_parameter<double>("vision_timeout_s", 1.5);
    declare_parameter<double>("payload_timeout_s", 2.0);
    declare_parameter<double>("mission_status_timeout_s", 2.0);
    declare_parameter<double>("mission_timeout_s", 900.0);
    declare_parameter<double>("startup_grace_s", 20.0);
    declare_parameter<int64_t>("minimum_free_megabytes", 512);

    flight_enable_ = get_parameter("flight_enable").as_bool();
    bundle_sha256_ = get_parameter("config_bundle_sha256").as_string();
    mavros_timeout_s_ = get_parameter("mavros_timeout_s").as_double();
    odom_timeout_s_ = get_parameter("odom_timeout_s").as_double();
    vision_timeout_s_ = get_parameter("vision_timeout_s").as_double();
    payload_timeout_s_ = get_parameter("payload_timeout_s").as_double();
    mission_status_timeout_s_ = get_parameter("mission_status_timeout_s").as_double();
    mission_timeout_s_ = get_parameter("mission_timeout_s").as_double();
    startup_grace_s_ = get_parameter("startup_grace_s").as_double();
    validate_timeouts();
    minimum_free_bytes_ = static_cast<std::uintmax_t>(
      std::max<int64_t>(0, get_parameter("minimum_free_megabytes").as_int())) *
      1024U * 1024U;
    validate_runtime_gate();
    open_log();
    open_recon_evidence_recorder();

    const auto now = Clock::now();
    last_mavros_ = now;
    last_odom_ = now;
    last_vision_ = now;
    last_payload_ = now;
    last_mission_ = now;
    mission_started_ = now;

    state_sub_ = create_subscription<mavros_msgs::msg::State>(
      "/mavros/state", rclcpp::QoS(10).reliable(),
      [this](const mavros_msgs::msg::State::SharedPtr message) {
        mavros_seen_ = true;
        connected_ = message->connected;
        armed_ = message->armed;
        mode_ = message->mode;
        last_mavros_ = Clock::now();
      });
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/mavros/local_position/odom", rclcpp::SensorDataQoS(),
      [this](const nav_msgs::msg::Odometry::SharedPtr) {
        odom_seen_ = true;
        last_odom_ = Clock::now();
      });
    vision_sub_ = create_subscription<geometry_msgs::msg::PoseArray>(
      "/perception/drop_buckets_body", rclcpp::SensorDataQoS(),
      [this](const geometry_msgs::msg::PoseArray::SharedPtr) {
        vision_seen_ = true;
        last_vision_ = Clock::now();
      });
    payload_sub_ = create_subscription<cuadc_interfaces::msg::PayloadStatus>(
      "/cuadc/payload/status", rclcpp::QoS(10).reliable(),
      [this](const cuadc_interfaces::msg::PayloadStatus::SharedPtr message) {
        const bool valid_state =
          message->state <= cuadc_interfaces::msg::PayloadStatus::FAULT;
        const bool valid_idle_status = message->payload_index == 0U &&
          message->state == cuadc_interfaces::msg::PayloadStatus::UNKNOWN &&
          !message->busy;
        const bool valid_payload_status =
          valid_state &&
          (message->payload_index == 1U || message->payload_index == 2U);
        if (!valid_idle_status && !valid_payload_status) {
          invalid_payload_status_ = true;
          return;
        }
        payload_seen_ = true;
        payload_fault_ =
          message->state == cuadc_interfaces::msg::PayloadStatus::FAULT;
        last_payload_ = Clock::now();
      });
    mission_sub_ = create_subscription<cuadc_interfaces::msg::MissionStatus>(
      "/cuadc/mission/status", rclcpp::QoS(10).reliable(),
      [this](const cuadc_interfaces::msg::MissionStatus::SharedPtr message) {
        if (message->active && (!mission_seen_ || !mission_.active)) {
          mission_started_ = Clock::now();
        }
        mission_ = *message;
        mission_seen_ = true;
        last_mission_ = Clock::now();
        append_event(
          "mission_status", message->detail,
          message->state == cuadc_interfaces::msg::MissionStatus::ABORTED);
      });
    status_pub_ = create_publisher<cuadc_interfaces::msg::SafetyStatus>(
      "/cuadc/safety/status", rclcpp::QoS(10).reliable().transient_local());
    timer_ = create_wall_timer(500ms, std::bind(&SafetyMonitor::tick, this));
  }

private:
  using Clock = std::chrono::steady_clock;

  static double age(const Clock::time_point & stamp)
  {
    return std::chrono::duration<double>(Clock::now() - stamp).count();
  }

  void validate_timeouts()
  {
    const std::array<double, 7> values{
      mavros_timeout_s_, odom_timeout_s_, vision_timeout_s_, payload_timeout_s_,
      mission_status_timeout_s_, mission_timeout_s_, startup_grace_s_};
    if (!std::all_of(values.begin(), values.end(), [](const double value) {
        return std::isfinite(value) && value > 0.0;
      }))
    {
      throw std::invalid_argument("safety timeouts and startup_grace_s must be finite and positive");
    }
  }

  void validate_runtime_gate()
  {
    const char * gate = std::getenv("CUADC_REAL_RUNTIME_GATE");
    const char * hash = std::getenv("CUADC_CONFIG_BUNDLE_SHA256");
    gate_open_ = gate != nullptr &&
      std::string(gate) == "CUADC_FLIGHT_LAUNCH_PREFLIGHT_PASSED" &&
      hash != nullptr && std::string(hash) == bundle_sha256_ &&
      bundle_sha256_.size() == 64U;
    if (flight_enable_ && !gate_open_) {
      fault_latched_ = true;
      RCLCPP_FATAL(get_logger(), "Flight runtime gate or bundle SHA is invalid");
    }
  }

  void open_log()
  {
    std::string directory = get_parameter("log_directory").as_string();
    if (directory.empty()) {
      const char * home = std::getenv("HOME");
      directory = std::string(home == nullptr ? "/tmp" : home) + "/cuadc_logs";
    }
    log_directory_ = std::filesystem::absolute(directory);
    std::filesystem::create_directories(log_directory_);
    const auto timestamp = std::chrono::system_clock::now().time_since_epoch().count();
    log_path_ = log_directory_ / ("flight_events_" + std::to_string(timestamp) + ".jsonl");
    log_.open(log_path_, std::ios::out | std::ios::app);
    if (!log_.is_open()) {
      recorder_fault_ = true;
      RCLCPP_ERROR(get_logger(), "Cannot open event log: %s", log_path_.c_str());
    } else {
      RCLCPP_INFO(get_logger(), "Event log: %s", log_path_.c_str());
    }
  }

  void open_recon_evidence_recorder()
  {
    try {
      std::filesystem::path directory =
        get_parameter("recon_evidence_directory").as_string();
      if (directory.empty()) {
        directory = log_directory_ / "recon_evidence";
      }
      cuadc_recon::EvidenceRecorderOptions options;
      options.directory = std::move(directory);
      options.image_topic = get_parameter("recon_image_topic").as_string();
      options.minimum_interval_s = get_parameter("recon_capture_interval_s").as_double();
      const auto jpeg_quality = get_parameter("recon_jpeg_quality").as_int();
      if (jpeg_quality < 1 || jpeg_quality > 100) {
        throw std::invalid_argument("recon_jpeg_quality must be in [1, 100]");
      }
      options.jpeg_quality = static_cast<int>(jpeg_quality);
      recon_recorder_ =
        std::make_unique<cuadc_recon::ReconEvidenceRecorder>(*this, std::move(options));
      if (!recon_recorder_->healthy()) {
        recorder_fault_ = true;
        recorder_detail_ = recon_recorder_->detail();
      }
    } catch (const std::exception & error) {
      recorder_fault_ = true;
      recorder_detail_ = std::string("recon_recorder_init_failed:") + error.what();
      RCLCPP_ERROR(get_logger(), "%s", recorder_detail_.c_str());
    }
  }

  static std::string json_escape(const std::string & value)
  {
    std::ostringstream output;
    for (const char character : value) {
      switch (character) {
        case '\\': output << "\\\\"; break;
        case '"': output << "\\\""; break;
        case '\n': output << "\\n"; break;
        case '\r': output << "\\r"; break;
        case '\t': output << "\\t"; break;
        default: output << character; break;
      }
    }
    return output.str();
  }

  void append_event(
    const std::string & type, const std::string & detail, bool critical)
  {
    if (!log_.is_open()) {
      return;
    }
    const double steady_s = std::chrono::duration<double>(
      Clock::now() - started_).count();
    log_ << "{\"sequence\":" << sequence_++ <<
      ",\"steady_s\":" << std::fixed << std::setprecision(3) << steady_s <<
      ",\"type\":\"" << json_escape(type) << "\"" <<
      ",\"critical\":" << (critical ? "true" : "false") <<
      ",\"state\":" << static_cast<unsigned int>(mission_.state) <<
      ",\"detail\":\"" << json_escape(detail) << "\"}\n";
    log_.flush();
    if (!log_) {
      recorder_fault_ = true;
    }
  }

  bool vision_required() const
  {
    if (!mission_seen_ || !mission_.active) {
      return true;
    }
    return mission_.state == cuadc_interfaces::msg::MissionStatus::TAKEOFF ||
      mission_.state == cuadc_interfaces::msg::MissionStatus::BASKET_SEARCH ||
      mission_.state == cuadc_interfaces::msg::MissionStatus::PAYLOAD_DROP;
  }

  bool payload_ready() const
  {
    return payload_seen_;
  }

  bool payload_stale() const
  {
    return !payload_seen_ || age(last_payload_) > payload_timeout_s_;
  }

  bool payload_fault() const
  {
    return invalid_payload_status_ || payload_fault_;
  }

  void tick()
  {
    bool critical = fault_latched_;
    std::string detail = gate_open_ ? "healthy" : "runtime_gate_closed";
    const bool mission_active = mission_seen_ && mission_.active;
    const double uptime_s = std::chrono::duration<double>(Clock::now() - started_).count();
    const bool mavros_fresh = mavros_seen_ && connected_ &&
      age(last_mavros_) <= mavros_timeout_s_;
    const bool odom_fresh = odom_seen_ && age(last_odom_) <= odom_timeout_s_;
    const bool vision_fresh = !vision_required() ||
      (vision_seen_ && age(last_vision_) <= vision_timeout_s_);
    const bool payload_fresh = payload_ready() &&
      age(last_payload_) <= payload_timeout_s_;
    const bool mission_fresh = mission_seen_ &&
      age(last_mission_) <= mission_status_timeout_s_;
    const bool startup_dependencies_ready = mavros_fresh && odom_fresh &&
      vision_fresh && payload_fresh && mission_fresh;
    // A single launch necessarily starts this process before all ROS publishers
    // are matched. During this bounded steady-clock grace period report BLOCKED,
    // but do not permanently latch a transport fault. The exemption ends early
    // once every dependency has produced data, and never applies once armed.
    const bool startup_blocked = flight_enable_ && !armed_ &&
      uptime_s < startup_grace_s_ && !startup_dependencies_ready;
    if (startup_blocked) {
      detail = "startup_grace_waiting_for_dependencies";
    } else if (flight_enable_ && !startup_dependencies_ready) {
      critical = true;
      if (!mavros_fresh) {
        detail = "mavros_state_stale_or_disconnected";
      } else if (!odom_fresh) {
        detail = "odometry_stale";
      } else if (!vision_fresh) {
        detail = "basket_perception_stale_before_second_release";
      } else if (!payload_fresh) {
        detail = "payload_status_stale";
      } else {
        detail = "mission_status_stale";
      }
    } else if (mission_active && (!odom_seen_ || age(last_odom_) > odom_timeout_s_)) {
      critical = true;
      detail = "odometry_stale";
    } else if (mission_active && vision_required() &&
      (!vision_seen_ || age(last_vision_) > vision_timeout_s_))
    {
      critical = true;
      detail = "basket_perception_stale_before_second_release";
    } else if (mission_active && payload_stale()) {
      critical = true;
      detail = "payload_status_stale";
    } else if (payload_fault()) {
      critical = true;
      detail = "payload_fault";
    } else if (mission_active && age(last_mission_) > mission_status_timeout_s_) {
      critical = true;
      detail = "mission_status_stale";
    } else if (mission_active && age(mission_started_) > mission_timeout_s_)
    {
      critical = true;
      detail = "mission_timeout";
    }
    try {
      const auto space = std::filesystem::space(log_directory_);
      if (space.available < minimum_free_bytes_) {
        recorder_fault_ = true;
      }
    } catch (const std::exception &) {
      recorder_fault_ = true;
    }
    if (!recon_recorder_ || !recon_recorder_->healthy()) {
      recorder_fault_ = true;
      if (recon_recorder_) {
        recorder_detail_ = recon_recorder_->detail();
      } else if (recorder_detail_.empty()) {
        recorder_detail_ = "recon_recorder_unavailable";
      }
    }
    if (critical) {
      fault_latched_ = true;
    }

    cuadc_interfaces::msg::SafetyStatus status;
    status.header.stamp = now();
    const bool blocked = startup_blocked || !gate_open_;
    status.level = fault_latched_ ? cuadc_interfaces::msg::SafetyStatus::FAULT :
      (blocked ? cuadc_interfaces::msg::SafetyStatus::BLOCKED :
      (recorder_fault_ ? cuadc_interfaces::msg::SafetyStatus::DEGRADED :
      cuadc_interfaces::msg::SafetyStatus::SAFE));
    status.flight_enable = flight_enable_;
    status.runtime_gate_open = gate_open_;
    status.mavros_command_ready = gate_open_ && mavros_fresh &&
      !startup_blocked && !fault_latched_;
    status.busy = mission_active;
    status.interlock_active = startup_blocked || fault_latched_ || !gate_open_;
    status.fault_latched = fault_latched_;
    status.detail = recorder_fault_ ? detail + ";recorder_degraded" : detail;
    if (recorder_fault_ && !recorder_detail_.empty()) {
      status.detail += ";" + recorder_detail_;
    }
    status_pub_->publish(status);
    append_event("safety_status", status.detail, fault_latched_);
  }

  bool flight_enable_{false};
  bool gate_open_{false};
  bool connected_{false};
  bool armed_{false};
  bool mavros_seen_{false};
  bool odom_seen_{false};
  bool vision_seen_{false};
  bool mission_seen_{false};
  bool payload_seen_{false};
  bool payload_fault_{false};
  bool invalid_payload_status_{false};
  bool fault_latched_{false};
  bool recorder_fault_{false};
  std::string bundle_sha256_;
  std::string mode_;
  std::string recorder_detail_;
  double mavros_timeout_s_{1.5};
  double odom_timeout_s_{0.6};
  double vision_timeout_s_{1.5};
  double payload_timeout_s_{2.0};
  double mission_status_timeout_s_{2.0};
  double mission_timeout_s_{900.0};
  double startup_grace_s_{20.0};
  std::uintmax_t minimum_free_bytes_{512U * 1024U * 1024U};
  std::uint64_t sequence_{0U};
  Clock::time_point started_;
  Clock::time_point last_mavros_;
  Clock::time_point last_odom_;
  Clock::time_point last_vision_;
  Clock::time_point last_payload_;
  Clock::time_point last_mission_;
  Clock::time_point mission_started_;
  cuadc_interfaces::msg::MissionStatus mission_;
  std::filesystem::path log_directory_;
  std::filesystem::path log_path_;
  std::ofstream log_;
  std::unique_ptr<cuadc_recon::ReconEvidenceRecorder> recon_recorder_;
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr vision_sub_;
  rclcpp::Subscription<cuadc_interfaces::msg::PayloadStatus>::SharedPtr payload_sub_;
  rclcpp::Subscription<cuadc_interfaces::msg::MissionStatus>::SharedPtr mission_sub_;
  rclcpp::Publisher<cuadc_interfaces::msg::SafetyStatus>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SafetyMonitor>());
  rclcpp::shutdown();
  return 0;
}
