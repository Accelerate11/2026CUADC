#define CUADC_TIMING_TEST
#include "../src/cuadc_full_mission_node.cpp"
#include <stdexcept>

struct StageProfileIntegrationTest
{
  static void check(bool good, const char * message)
  {if (!good) {throw std::runtime_error(message);}}

  static void run()
  {
    using Stage = cuadc_guided::MotionStage;
#ifdef CUADC_SOURCE_ROOT
    for (const std::string relative : {"full_mission/mission.yaml", "drop_only/mission.yaml",
      "src/cuadc_full_mission_20260803/config/mission_real.yaml"})
    {
      rclcpp::NodeOptions options;
      options.arguments({"--ros-args", "--params-file", std::string(CUADC_SOURCE_ROOT)+"/"+relative});
      auto loaded = std::make_shared<CuadcFullMissionNode>(options);
      loaded->timer_->cancel();
      check(!loaded->flight_enable_ && !loaded->auto_arm_on_guided_ &&
        loaded->guided_local_profiles_enabled_ && !loaded->guided_fcu_config_enabled_,
        "actual ROS mission YAML did not retain safe proposal defaults");
      for (const auto & profile : loaded->guided_fcu_profiles_) {
        check(profile.param_ids.empty() && profile.param_values.empty(),
          "default YAML unexpectedly contains FCU gain mutations");
      }
    }
#endif
    auto node = std::make_shared<CuadcFullMissionNode>();
    auto & n = *node;
    n.timer_->cancel();
    // No executor or actuator service runs in this integration test.
    n.flight_enable_ = n.config_valid_ = n.guided_active_ = n.frame_locked_ = true;
    n.fcu_state_.connected = true;
    n.fcu_state_.armed = true;
    n.fcu_state_.mode = "GUIDED";
    n.have_odom_ = true;
    n.last_odom_arrival_ = SteadyClock::now();
    n.home_ = Point3{};
    n.position_ = {0.0, 0.0, 3.0};
    n.target_ = {30.0, 0.0, 3.0};
    n.state_ = State::SEARCH;
    n.search_pass_ = 1;
    n.search_index_ = 0U;
    check(n.motion_stage() == Stage::CRUISE, "first ingress not cruise");
    n.search_index_ = 1U;
    check(n.motion_stage() == Stage::SEARCH, "sweep not search");
    n.state_ = State::ALIGN;
    n.inter_target_transit_active_ = true;
    check(n.motion_stage() == Stage::APPROACH, "target transit not approach");
    n.inter_target_transit_active_ = false;
    check(n.motion_stage() == Stage::ALIGN, "fine alignment not align");
    n.state_ = State::RECON_SURVEY;
    check(n.motion_stage() == Stage::RECON, "recon stage missing");
    n.state_ = State::RETURN_HOME;
    check(n.motion_stage() == Stage::RETURN, "return stage missing");
    n.state_ = State::RELEASE;
    check(n.motion_stage() == Stage::RELEASE, "release stage missing");

    n.release_stabilization_since_ = SteadyClock::now()-10s;
    n.release_gate_start_ros_ns_ = 123;
    n.release_gate_.count = 3;
    n.guided_reference_.reset({0.0,0.0,3.0});
    n.guided_reference_needs_reset_ = false;
    for (int i=0;i<30;++i) {
      n.guided_reference_.step({30.0,0.0,3.0}, n.active_motion_profile(),0.05,false);
    }
    n.update_guided_profile_gate();
    check(n.guided_switch_active_, "release stage gate not started");
    check(!n.release_stabilization_since_ && n.release_gate_.count == 0 &&
      n.release_gate_start_ros_ns_ == 0, "stage wait counted toward forced release timer");
    for (int i=0;i<100 && !n.guided_reference_.stopped();++i) {
      n.guided_reference_.step({},n.active_motion_profile(),0.05,true);
    }
    check(n.guided_reference_.stopped(), "reference did not stop before apply");
    n.guided_switch_start_ = SteadyClock::now()-1s;
    n.last_odom_arrival_ = SteadyClock::now()-1s;
    n.update_guided_profile_gate();
    check(n.guided_switch_active_, "stale physical nav opened stage gate");
    n.last_odom_arrival_ = SteadyClock::now();
    n.horizontal_speed_m_s_ = 1.0;
    n.update_guided_profile_gate();
    check(n.guided_switch_active_, "moving aircraft opened stage gate");
    n.horizontal_speed_m_s_ = n.vertical_speed_m_s_ = 0.0;
    n.update_guided_profile_gate();
    check(n.guided_switch_active_, "one settled sample bypassed dwell");
    n.guided_switch_start_ = SteadyClock::now()-2s;
    n.update_guided_profile_gate();
    check(!n.guided_switch_active_ && n.guided_applied_motion_stage_ == Stage::RELEASE,
      "stable physical nav did not apply local release profile");
    check(!n.release_stabilization_since_, "gate resume inherited forced release deadline");

    n.active_bucket_ = BucketTrack{};
    n.active_bucket_->rim_body = {1.0,0.0,0.0};
    n.release_offsets_frd_ = {0.0,0.0,0.0,0.0,0.0,0.0};
    n.position_ = {};
    n.current_roll_ = n.current_pitch_ = n.mission_yaw_ = 0.0;
    const auto limited = n.visual_release_target(1.0);
    const double error = n.visual_release_alignment_error();
    check(std::abs(limited.x-0.35)<1e-10 && std::abs(error-1.0)<1e-10,
      "vision gain altered actual release error or did not scale reference correction");
    n.guided_local_profiles_enabled_ = false;
    check(std::abs(n.visual_release_target(1.0).x-1.0)<1e-10, "disabled profile changed V16 feedback");
    n.guided_local_profiles_enabled_ = true;

    n.enter(State::TAKEOFF);
    check(!n.guided_applied_motion_stage_ && n.guided_reference_needs_reset_,
      "takeoff controller did not invalidate entry profile");
    n.position_ = {4.0,5.0,3.0};
    n.state_ = State::SEARCH;
    n.search_index_ = 0U;
    n.segment_.end = {30.0,5.0,3.0};
    n.last_odom_arrival_ = SteadyClock::now();
    n.update_guided_profile_gate();
    check(n.guided_switch_active_ && n.guided_pending_motion_stage_ == Stage::CRUISE &&
      std::abs(n.guided_reference_.position().z-3.0)<1e-10,
      "first airborne position stream reused ground reference/profile");
    n.guided_switch_start_ = SteadyClock::now()-1s;
    n.update_guided_profile_gate();
    n.guided_switch_start_ = SteadyClock::now()-2s;
    n.update_guided_profile_gate();
    check(!n.guided_switch_active_ && distance_xyz(n.segment_.start,n.position_)<1e-10 &&
      distance_xyz(n.target_,n.segment_.start)<1e-10 &&
      std::abs(n.segment_.duration_s-26.0/3.0)<1e-8,
      "entry segment did not rebase at held airborne reference under cruise speed");
    n.state_ = State::RETURN_CLIMB;
    n.update_guided_profile_gate();
    check(n.guided_switch_active_ && n.guided_pending_motion_stage_ == Stage::RETURN,
      "watchdog stage change retained obsolete cruise gate");
    n.state_ = State::SEARCH;
    n.search_index_ = 1U;
    n.update_guided_profile_gate();
    check(n.guided_switch_active_ && n.guided_pending_motion_stage_ == Stage::SEARCH,
      "replacement stage retained a superseded return gate");
    n.state_ = State::RETURN_CLIMB;
    n.update_guided_profile_gate();
    n.guided_switch_start_ = SteadyClock::now()-21s;
    n.last_odom_arrival_ = SteadyClock::now()-1s;
    n.update_guided_profile_gate();
    check(n.state_ == State::LAND && !n.guided_switch_active_ && n.mission_failed_,
      "bounded switch timeout did not terminate mission via LAND");
    // Arm loss is checked before any FCU profile dispatch, even while still
    // reporting GUIDED. Only WAIT_ARM may configure while disarmed.
    n.state_ = State::SEARCH;
    n.last_odom_arrival_ = SteadyClock::now();
    n.guided_fcu_config_enabled_ = true;
    n.update_guided_profile_gate();
    check(n.guided_switch_active_ && !n.guided_fcu_request_started_,
      "disarm fixture failed to enter an undispatched stage gate");
    n.fcu_state_.armed = false;
    n.update_guided_profile_gate();
    check(!n.guided_switch_active_ && !n.guided_fcu_request_started_ &&
      !n.guided_profile_client_->busy(), "disarmed airborne state dispatched FCU configuration");
    check(!n.flight_gate_ok() && n.state_ == State::ABORT,
      "disarmed airborne state did not retain V16 abort behavior");
    n.fcu_state_.armed = true;
    n.state_ = State::RELEASE;
    n.release_actuation_committed_ = true;
    n.release_open_time_ = SteadyClock::now()-200ms;
    n.release_stabilization_since_ = SteadyClock::now()-1s;
    const auto actuator_hold_start = n.release_open_time_;
    const auto stabilization_start = n.release_stabilization_since_;
    PendingServoCommand pending;
    pending.purpose = ServoPurpose::RELEASE;
    pending.sent = SteadyClock::now()-100ms;
    const auto ack_start = pending.sent;
    n.pending_servo_commands_.push_back(std::move(pending));
    n.guided_applied_motion_stage_ = Stage::CRUISE;
    n.update_guided_profile_gate();
    check(!n.guided_switch_active_ && n.release_open_time_ == actuator_hold_start &&
      n.release_stabilization_since_ == stabilization_start &&
      n.pending_servo_commands_.back().sent == ack_start,
      "committed release stage gate paused or reset actuator deadlines");
    n.pending_servo_commands_.clear();
    n.release_actuation_committed_ = false;
    n.state_ = State::SEARCH;
    n.publish_setpoint_ = true;
    n.update_guided_profile_gate();
    check(n.guided_switch_active_, "handoff fixture did not enter stage wait");
    auto handoff = std::make_shared<mavros_msgs::msg::State>(n.fcu_state_);
    handoff->mode = "LOITER";
    n.state_callback(handoff);
    check(n.state_ == State::PILOT_OVERRIDE && !n.publish_setpoint_ &&
      !n.guided_switch_active_ && !n.guided_profile_client_->busy(),
      "mode handoff did not cancel stage wait and relinquish setpoint stream");
    std::cout << "stage selection, physical gate, release timer and takeover integration passed\n";
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc,argv);
  try {StageProfileIntegrationTest::run();}
  catch (const std::exception & e) {std::cerr << e.what() << '\n'; rclcpp::shutdown(); return 1;}
  rclcpp::shutdown();
  return 0;
}
