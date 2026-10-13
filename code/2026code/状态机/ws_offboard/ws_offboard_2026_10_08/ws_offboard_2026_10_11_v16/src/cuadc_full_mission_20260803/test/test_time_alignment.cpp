#define CUADC_TIMING_TEST
#include "../src/cuadc_full_mission_node.cpp"
#include <stdexcept>
#include <thread>

struct TimeSyncTest
{
  static void monitor()
  {
    auto n = std::make_shared<CuadcFullMissionNode>();
    // Exercise production callbacks on telemetry, but never execute the
    // mission timer, publish setpoints, or call actuator services.
    n->timer_->cancel();
    const auto start = SteadyClock::now();
    std::size_t aligned = 0U;
    bool reported = false;
    while (rclcpp::ok() && std::chrono::duration<double>(SteadyClock::now()-start).count()<65) {
      rclcpp::spin_some(n);
      if (!reported && n->flight_clock_ready()) {
        std::cout << "FIRST_CLOCK_READY elapsed_s="
                  << std::chrono::duration<double>(SteadyClock::now()-start).count()
                  << std::endl;
        reported = true;
      }
      check(!n->fcu_state_.armed, "aircraft armed during diagnostic");
      if (n->navigation_sample_at(n->now()-rclcpp::Duration::from_seconds(0.1))) {++aligned;}
      std::this_thread::sleep_for(10ms);
    }
    std::cout << "LIVE_CALLBACK_CHECK ready=" << n->flight_clock_ready()
              << " synchronized_lookups=" << aligned << "\n";
    check(n->flight_clock_ready() && aligned>100, "live time alignment did not become ready");
  }
  static void check(bool good, const char * message)
  {
    if (!good) {throw std::runtime_error(message);}
  }
  static void run()
  {
    {
      ReleaseGate g;
      using D = ReleaseGate::Decision;
      check(g.update(0.49,1,true,0.01,0,true)==D::WAIT, "hold bypassed");
      check(g.update(0.51,2,true,0.03,0.09,true)==D::WAIT, "first frame released");
      check(g.update(0.55,2,true,0.01,0.09,true)==D::WAIT && g.count==1, "duplicate counted");
      check(g.update(0.6,3,true,0.049,0.09,true)==D::WAIT, "second frame released");
      check(g.update(0.7,4,true,0.04,0.09,true)==D::WAIT, "third frame released");
      check(g.update(0.8,5,true,0.04,0.09,true)==D::STABLE, "four frames rejected");
      g.reset(); g.update(1,1,true,0.02,0.09,true);
      g.update(1.01,1,true,0.02,0.1,true);
      check(g.count==0, "speed violation between frames ignored");
      g.update(1.1,2,true,0.04,0.09,true);
      check(g.count==0, "sequence started above 3cm");
      g.update(1.2,3,true,0.02,0.09,true); g.update(1.3,4,true,0.05,0.09,true);
      check(g.count==0, "5cm boundary accepted");
      check(g.update(4,5,true,0.03,0.5,true)==D::RELAXED, "relaxed gate kept speed limit");
      check(g.update(4.1,5,true,0.02,0,true)==D::WAIT, "relaxed duplicate accepted");
      check(g.update(5,6,false,0.02,0,true)==D::WAIT, "stale frame accepted");
      check(g.update(5.9,7,true,0.031,0,true)==D::WAIT, "relaxed error boundary wrong");
      check(g.update(6,7,false,1,1,false)==D::FORCED, "deadline extended");
      g.reset();
      check(g.update(0.6,8,true,0.01,0,false)==D::WAIT && g.count==0, "pre-hold exposure counted");
    }
    // No executor is spun: the timer cannot call flight or servo services.
    auto node = std::make_shared<CuadcFullMissionNode>();
    auto & n = *node;
    n.measurement_time_sync_ = true;
    n.timesync_config_valid_ = true;
    n.timesync_good_samples_ = 511;
    n.timesync_arrival_ = SteadyClock::now();
    const auto t = n.now();
    n.navigation_history_ = {
      {t-rclcpp::Duration::from_seconds(0.1), {0,0,0}, 0,0,0},
      {t, {0.1,0,0}, 0,0,0}};
    n.attitude_history_ = {
      {t-rclcpp::Duration::from_seconds(0.1), {}, 0.1,0.2,3.13},
      {t, {}, 0.3,0.4,-3.13}};
    auto mid = n.navigation_sample_at(t-rclcpp::Duration::from_seconds(0.05));
    check(mid.has_value(), "bracketed sample missing");
    check(std::abs(mid->position.x-0.05)<1e-9, "position interpolation failed");
    check(std::abs(mid->roll-0.2)<1e-9, "independent attitude interpolation failed");
    check(std::abs(std::abs(mid->yaw)-kPi)<1e-9, "angle wrap interpolation failed");
    check(!n.navigation_sample_at(t+rclcpp::Duration::from_seconds(0.01)), "extrapolation accepted");
    n.attitude_history_.clear();
    check(!n.navigation_sample_at(t), "missing IMU accepted");

    // Identical old observations must not move the target as aircraft moves.
    n.active_bucket_.emplace();
    n.active_bucket_->rim_body = {0.5,0.2,-1};
    n.active_bucket_->capture_navigation = NavigationSample{t, {0,0,0}, 0,0,0};
    n.active_bucket_->stamp = t;
    n.home_ = Point3{};
    n.payload_index_ = 0;
    n.release_offsets_frd_ = {0,0,0};
    n.mission_yaw_ = 0;
    n.current_roll_ = n.current_pitch_ = 0;
    n.position_ = {};
    const auto first = n.visual_release_target(1);
    n.position_ = {0.3,0.1,0};
    const auto second = n.visual_release_target(1);
    check(distance_xy(first, second)<1e-9, "old frame chased current aircraft position");
    check(std::abs(n.visual_release_alignment_error()-std::hypot(0.2,0.1))<1e-9,
      "release gate did not use compensated error");
    n.active_bucket_->stamp = t-rclcpp::Duration::from_seconds(1);
    check(std::isinf(n.visual_release_alignment_error()), "old frame allowed early release");

    // Zero delay preserves existing crosshair geometry at nonzero attitude.
    n.position_ = {4,5,6}; n.mission_yaw_ = 1.2; n.current_vehicle_yaw_ = 1.1;
    n.current_roll_ = 0.1; n.current_pitch_ = -0.2;
    n.active_bucket_->capture_navigation = NavigationSample{
      t,n.position_,n.current_roll_,n.current_pitch_,n.current_vehicle_yaw_};
    check(distance_xyz(n.release_rim_at_current_pose(), n.active_bucket_->rim_body)<1e-9,
      "zero-delay projection changed geometry");
    n.position_ = {}; n.current_roll_ = n.current_pitch_ = 0;
    n.active_bucket_->capture_navigation = NavigationSample{t,{},0,0,0};
    n.current_vehicle_yaw_ = kPi/2;
    const auto rotated_target = n.visual_release_target(1);
    check(distance_xy(rotated_target, first)<1e-9, "yaw motion moved fixed target");

    // FCU timestamp, not callback arrival, is the history key.
    n.navigation_history_.clear();
    auto odom = std::make_shared<nav_msgs::msg::Odometry>();
    odom->header.stamp = t-rclcpp::Duration::from_seconds(0.05);
    odom->pose.pose.orientation.w=1;
    n.odom_callback(odom);
    check(n.navigation_history_.size()==1, "valid odometry rejected");
    check(n.navigation_history_.back().stamp == rclcpp::Time(odom->header.stamp),
      "odometry still uses receipt time");
    n.odom_callback(odom);
    check(n.navigation_history_.size()==1, "duplicate odometry accepted");
    odom->header.stamp=t-rclcpp::Duration::from_seconds(2);
    n.odom_callback(odom);
    check(n.navigation_history_.size()==1, "stale odometry accepted");
    n.timesync_good_samples_=500;
    check(!n.flight_clock_ready(), "preconvergence fallback accepted");
    check(!n.vision_ready_before_takeoff(), "unsynchronized takeoff gate opened");
    // Preparation and the original prestream wait finish BEFORE WAIT_GUIDED.
    // On the pilot's switch the next tick goes directly to WAIT_ARM.
    n.timesync_good_samples_=511; n.timesync_arrival_=SteadyClock::now();
    n.config_valid_=true; n.release_mode_="virtual"; n.auto_arm_on_guided_=false;
    n.fcu_state_.connected=true; n.fcu_state_.armed=false; n.guided_active_=false;
    n.configured_geographic_heading_deg_=212.05;
    n.position_stable_since_=SteadyClock::now()-3s;
    n.have_vision_message_=true; n.have_aligned_vision_=true;
    n.last_vision_message_arrival_=n.last_aligned_vision_arrival_=SteadyClock::now();
    n.vision_heartbeat_count_=n.vision_message_count_=3;
    n.state_=State::WAIT_FCU;
    n.tick(); check(n.state_==State::PRESTREAM, "preparation not before WAIT_GUIDED");
    n.state_enter_time_=SteadyClock::now()-2s;
    n.tick(); check(n.state_==State::WAIT_GUIDED, "ready state missing");
    const auto stable_since=n.position_stable_since_;
    n.position_={7,8,9}; n.guided_active_=true;
    n.tick(); check(n.state_==State::WAIT_ARM, "GUIDED started another preflight wait");
    check(n.position_stable_since_==stable_since, "GUIDED reset stability timer");
    check(distance_xyz(*n.home_,n.position_)<1e-9, "home not refreshed at final transition");
    n.state_=State::WAIT_GUIDED; n.guided_active_=false;
    n.timesync_arrival_=SteadyClock::now()-2s;
    n.tick(); check(n.state_==State::WAIT_FCU, "stale readiness still advertised");
    n.timesync_arrival_=SteadyClock::now(); n.guided_active_=true;
    n.tick(); n.state_enter_time_=SteadyClock::now()-2s; n.tick();
    check(n.state_==State::WAIT_GUIDED, "early GUIDED preparation failed");
    n.tick(); check(n.state_==State::WAIT_GUIDED, "early GUIDED caused delayed auto-arm");
    n.guided_active_=false; n.tick(); n.guided_active_=true; n.tick();
    check(n.state_==State::WAIT_ARM, "fresh GUIDED edge was not accepted");
    std::cout << "Time alignment regression checks passed\n";
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc,argv);
  try {
    if (argc>1 && std::string(argv[1])=="--clock-monitor") {TimeSyncTest::monitor();}
    else {TimeSyncTest::run();}
  } catch (...) {rclcpp::shutdown(); throw;}
  rclcpp::shutdown();
}
