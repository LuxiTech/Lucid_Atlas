#include <chrono>
#include <algorithm>
#include <memory>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "go2_cmd_vel_bridge/cmd_vel_adapter.hpp"
#include "rclcpp/rclcpp.hpp"
#include "unitree_api/msg/request.hpp"
#include "unitree_go/msg/sport_mode_state.hpp"

namespace go2_cmd_vel_bridge
{
namespace
{
using namespace std::chrono_literals;
}  // namespace

class Go2CmdVelBridgeNode : public rclcpp::Node
{
public:
  Go2CmdVelBridgeNode()
  : Node("go2_cmd_vel_bridge"),
    adapter_(readLimits())
  {
    cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    request_topic_ = declare_parameter<std::string>("request_topic", "/api/sport/request");
    motion_switcher_topic_ = declare_parameter<std::string>(
      "motion_switcher_topic", "/api/motion_switcher/request");
    robot_state_topic_ = declare_parameter<std::string>(
      "robot_state_topic", "/api/robot_state/request");
    sport_state_topic_ = declare_parameter<std::string>("sport_state_topic", "/lf/sportmodestate");
    expected_sender_ip_ = declare_parameter<std::string>("expected_sender_ip", "192.168.123.51");
    cmd_timeout_sec_ = declare_parameter<double>("cmd_timeout_sec", 0.2);
    publish_stop_on_timeout_ = declare_parameter<bool>("publish_stop_on_timeout", true);
    stop_burst_count_ = declare_parameter<int>("stop_burst_count", 12);
    qos_depth_ = declare_parameter<int>("qos_depth", 1);
    auto_enable_sport_mode_ = declare_parameter<bool>("auto_enable_sport_mode", true);
    enable_period_sec_ = declare_parameter<double>("enable_period_sec", 2.0);
    desired_motion_mode_ = declare_parameter<int>("desired_motion_mode", 1);
    desired_gait_type_ = declare_parameter<int>("desired_gait_type", 1);
    enforce_desired_motion_mode_ = declare_parameter<bool>("enforce_desired_motion_mode", true);
    motion_mode_request_period_sec_ = declare_parameter<double>("motion_mode_request_period_sec", 1.0);

    request_pub_ = create_publisher<unitree_api::msg::Request>(
      request_topic_, rclcpp::QoS(static_cast<size_t>(qos_depth_)).best_effort());
    motion_switcher_pub_ = create_publisher<unitree_api::msg::Request>(
      motion_switcher_topic_, rclcpp::QoS(static_cast<size_t>(qos_depth_)).best_effort());
    robot_state_pub_ = create_publisher<unitree_api::msg::Request>(
      robot_state_topic_, rclcpp::QoS(static_cast<size_t>(qos_depth_)).best_effort());
    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_topic_, rclcpp::QoS(static_cast<size_t>(qos_depth_)),
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        onCmdVel(*msg);
      });
    sport_state_sub_ = create_subscription<unitree_go::msg::SportModeState>(
      sport_state_topic_, rclcpp::QoS(static_cast<size_t>(qos_depth_)).best_effort(),
      [this](const unitree_go::msg::SportModeState::SharedPtr msg) {
        onSportState(*msg);
      });

    watchdog_timer_ = create_wall_timer(100ms, [this]() {
      onWatchdog();
    });
    enable_timer_ = create_wall_timer(500ms, [this]() {
      onEnableTimer();
    });

    RCLCPP_INFO(
      get_logger(),
      "GO2 cmd_vel bridge started: %s -> %s, expected sender IP %s (ROS 2 does not expose "
      "publisher source IP to this node)",
      cmd_vel_topic_.c_str(), request_topic_.c_str(), expected_sender_ip_.c_str());
  }

private:
  VelocityLimits readLimits()
  {
    VelocityLimits limits;
    limits.max_linear_x = declare_parameter<double>("max_linear_x", limits.max_linear_x);
    limits.max_linear_y = declare_parameter<double>("max_linear_y", limits.max_linear_y);
    limits.max_angular_z = declare_parameter<double>("max_angular_z", limits.max_angular_z);
    limits.deadband = declare_parameter<double>("deadband", limits.deadband);
    return limits;
  }

  void onCmdVel(const geometry_msgs::msg::Twist & twist)
  {
    last_cmd_time_ = now();
    received_cmd_ = true;

    const auto request = adapter_.toRequest(twist, nextRequestId());
    if (request.header.identity.api_id == kSportApiIdMove) {
      ensureDesiredMotionMode();
    }
    request_pub_->publish(request);
    if (request.header.identity.api_id == kSportApiIdStopMove) {
      stopped_after_timeout_ = true;
      stop_burst_remaining_ = std::max(0, stop_burst_count_ - 1);
    } else {
      stopped_after_timeout_ = false;
      stop_burst_remaining_ = 0;
    }

    RCLCPP_DEBUG(
      get_logger(), "Published sport request api_id=%ld parameter=%s",
      request.header.identity.api_id, request.parameter.c_str());
  }

  void onSportState(const unitree_go::msg::SportModeState & state)
  {
    last_sport_state_time_ = now();
    seen_sport_state_ = true;
    latest_mode_ = state.mode;
    latest_gait_type_ = state.gait_type;

    // Go2 reports mode=0 when sport control is not ready. gait_type can be 0
    // while the robot is standing still and becomes non-zero during walking.
    sport_mode_ready_ = state.mode != 0;
    desired_motion_mode_ready_ =
      static_cast<int>(state.mode) == desired_motion_mode_ &&
      static_cast<int>(state.gait_type) == desired_gait_type_;
  }

  void onWatchdog()
  {
    if (stop_burst_remaining_ > 0) {
      publishStopRequest("stop burst");
      --stop_burst_remaining_;
      return;
    }

    if (!publish_stop_on_timeout_ || !received_cmd_ || stopped_after_timeout_) {
      return;
    }

    const double elapsed_sec = (now() - last_cmd_time_).seconds();
    if (elapsed_sec <= cmd_timeout_sec_) {
      return;
    }

    publishStopRequest("timeout");
    stopped_after_timeout_ = true;
    stop_burst_remaining_ = std::max(0, stop_burst_count_ - 1);

    RCLCPP_WARN(
      get_logger(), "No cmd_vel received for %.2fs, published StopMove request",
      elapsed_sec);
  }

  void onEnableTimer()
  {
    if (!auto_enable_sport_mode_) {
      return;
    }

    const auto current_time = now();
    if (received_cmd_ && (current_time - last_cmd_time_).seconds() <= cmd_timeout_sec_) {
      return;
    }
    if (seen_sport_state_ && sport_mode_ready_) {
      return;
    }
    if (last_enable_request_time_.nanoseconds() != 0 &&
      (current_time - last_enable_request_time_).seconds() < enable_period_sec_)
    {
      return;
    }

    publishEnableRequests();
    last_enable_request_time_ = current_time;
  }

  void publishStopRequest(const char * reason)
  {
    const auto request = adapter_.makeStopRequest(nextRequestId());
    request_pub_->publish(request);
    RCLCPP_DEBUG(get_logger(), "Published StopMove request (%s)", reason);
  }

  void ensureDesiredMotionMode()
  {
    if (!enforce_desired_motion_mode_) {
      return;
    }
    if (seen_sport_state_ && desired_motion_mode_ready_) {
      return;
    }

    const auto current_time = now();
    if (last_motion_mode_request_time_.nanoseconds() != 0 &&
      (current_time - last_motion_mode_request_time_).seconds() < motion_mode_request_period_sec_)
    {
      return;
    }

    publishEnableRequests();
    last_motion_mode_request_time_ = current_time;
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 3000,
      "Requested desired Go2 motion mode before Move: target mode=%d gait_type=%d, current mode=%u gait_type=%u",
      desired_motion_mode_, desired_gait_type_,
      static_cast<unsigned int>(latest_mode_),
      static_cast<unsigned int>(latest_gait_type_));
  }

  void publishEnableRequests()
  {
    motion_switcher_pub_->publish(makeRequest(1003, ""));
    robot_state_pub_->publish(makeRequest(1001, "{\"name\":\"sport_mode\",\"switch\":1}"));
    request_pub_->publish(makeRequest(1002, ""));
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Requested Go2 sport mode readiness (seen_state=%s mode=%u gait_type=%u)",
      seen_sport_state_ ? "true" : "false",
      static_cast<unsigned int>(latest_mode_),
      static_cast<unsigned int>(latest_gait_type_));
  }

  unitree_api::msg::Request makeRequest(const int64_t api_id, const std::string & parameter)
  {
    unitree_api::msg::Request request;
    request.header.identity.id = nextRequestId();
    request.header.identity.api_id = api_id;
    request.parameter = parameter;
    return request;
  }

  int64_t nextRequestId()
  {
    return request_id_++;
  }

  CmdVelAdapter adapter_;
  rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr request_pub_;
  rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr motion_switcher_pub_;
  rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr robot_state_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr sport_state_sub_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
  rclcpp::TimerBase::SharedPtr enable_timer_;
  rclcpp::Time last_cmd_time_;
  rclcpp::Time last_sport_state_time_;
  rclcpp::Time last_enable_request_time_;
  rclcpp::Time last_motion_mode_request_time_;
  std::string cmd_vel_topic_;
  std::string request_topic_;
  std::string motion_switcher_topic_;
  std::string robot_state_topic_;
  std::string sport_state_topic_;
  std::string expected_sender_ip_;
  double cmd_timeout_sec_{0.2};
  double enable_period_sec_{2.0};
  double motion_mode_request_period_sec_{1.0};
  bool publish_stop_on_timeout_{true};
  bool auto_enable_sport_mode_{true};
  bool enforce_desired_motion_mode_{true};
  bool received_cmd_{false};
  bool stopped_after_timeout_{true};
  bool seen_sport_state_{false};
  bool sport_mode_ready_{false};
  bool desired_motion_mode_ready_{false};
  uint8_t latest_mode_{0};
  uint8_t latest_gait_type_{0};
  int desired_motion_mode_{1};
  int desired_gait_type_{1};
  int qos_depth_{1};
  int stop_burst_count_{12};
  int stop_burst_remaining_{0};
  int64_t request_id_{1};
};

}  // namespace go2_cmd_vel_bridge

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<go2_cmd_vel_bridge::Go2CmdVelBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
