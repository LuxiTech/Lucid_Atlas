#include <algorithm>
#include <cmath>
#include <memory>
#include <sstream>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"

namespace
{

double clamp_value(const double value, const double limit)
{
  const double abs_limit = std::abs(limit);
  return std::clamp(value, -abs_limit, abs_limit);
}

std::string json_escape(const std::string & input)
{
  std::ostringstream out;
  for (const char ch : input) {
    if (ch == '\\') {
      out << "\\\\";
    } else if (ch == '"') {
      out << "\\\"";
    } else if (ch == '\n') {
      out << "\\n";
    } else if (ch != '\r') {
      out << ch;
    }
  }
  return out.str();
}

}  // namespace

class CommandGuardNode : public rclcpp::Node
{
public:
  CommandGuardNode()
  : Node("command_guard_node")
  {
    declare_parameter<std::string>("input_cmd_topic", "/web/cmd_vel");
    declare_parameter<std::string>("estop_topic", "/web/estop");
    declare_parameter<std::string>("goal_topic", "/web/goal_pose");
    declare_parameter<std::string>("accepted_cmd_topic", "/web/accepted_cmd_vel");
    declare_parameter<std::string>("control_status_topic", "/web/control_status");
    declare_parameter<bool>("enable_passthrough", false);
    declare_parameter<std::string>("output_cmd_topic", "/cmd_vel");
    declare_parameter<double>("max_linear_x", 0.5);
    declare_parameter<double>("max_linear_y", 0.5);
    declare_parameter<double>("max_angular_z", 0.5);
    declare_parameter<double>("linear_x_output_scale", 1.0);
    declare_parameter<double>("linear_y_output_scale", 1.0);
    declare_parameter<double>("angular_z_output_scale", 1.0);
    declare_parameter<double>("command_timeout_sec", 0.4);

    input_cmd_topic_ = get_parameter("input_cmd_topic").as_string();
    estop_topic_ = get_parameter("estop_topic").as_string();
    goal_topic_ = get_parameter("goal_topic").as_string();
    accepted_cmd_topic_ = get_parameter("accepted_cmd_topic").as_string();
    control_status_topic_ = get_parameter("control_status_topic").as_string();
    output_cmd_topic_ = get_parameter("output_cmd_topic").as_string();

    accepted_cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(accepted_cmd_topic_, 10);
    status_pub_ = create_publisher<std_msgs::msg::String>(
      control_status_topic_, rclcpp::QoS(1).transient_local().reliable());

    if (get_parameter("enable_passthrough").as_bool()) {
      output_cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(output_cmd_topic_, 10);
      RCLCPP_WARN(
        get_logger(),
        "enable_passthrough=true: accepted web commands will be forwarded to %s",
        output_cmd_topic_.c_str());
    }

    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      input_cmd_topic_, 10, std::bind(&CommandGuardNode::on_cmd, this, std::placeholders::_1));
    estop_sub_ = create_subscription<std_msgs::msg::Bool>(
      estop_topic_, 10, std::bind(&CommandGuardNode::on_estop, this, std::placeholders::_1));
    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      goal_topic_, 10, std::bind(&CommandGuardNode::on_goal, this, std::placeholders::_1));

    timer_ = create_wall_timer(
      std::chrono::milliseconds(100), std::bind(&CommandGuardNode::on_timer, this));

    publish_status(
      "ready",
      output_cmd_pub_ ? "网页控制保护节点已启动，速度指令会输出到底盘话题。"
                      : "网页控制保护节点已启动，演示模式不会输出到底盘。");
    RCLCPP_INFO(
      get_logger(), "command_guard_node started. input=%s accepted=%s estop=%s goal=%s",
      input_cmd_topic_.c_str(), accepted_cmd_topic_.c_str(), estop_topic_.c_str(),
      goal_topic_.c_str());
  }

private:
  void on_cmd(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    last_cmd_time_ = now();

    if (estop_active_) {
      publish_zero("estop", "急停已激活，忽略速度指令。");
      return;
    }

    geometry_msgs::msg::Twist guarded;
    guarded.linear.x = clamp_value(msg->linear.x, get_parameter("max_linear_x").as_double()) *
      get_parameter("linear_x_output_scale").as_double();
    guarded.linear.y = clamp_value(msg->linear.y, get_parameter("max_linear_y").as_double()) *
      get_parameter("linear_y_output_scale").as_double();
    guarded.angular.z = clamp_value(msg->angular.z, get_parameter("max_angular_z").as_double()) *
      get_parameter("angular_z_output_scale").as_double();

    accepted_cmd_pub_->publish(guarded);
    if (output_cmd_pub_) {
      output_cmd_pub_->publish(guarded);
    }
    zero_sent_after_timeout_ = false;

    std::ostringstream detail;
    detail << "accepted vx=" << guarded.linear.x << " vy=" << guarded.linear.y
           << " wz=" << guarded.angular.z;
    publish_status("manual_cmd", detail.str());
  }

  void on_estop(const std_msgs::msg::Bool::SharedPtr msg)
  {
    estop_active_ = msg->data;
    if (estop_active_) {
      publish_zero("estop", "急停已激活，已发布零速度。");
    } else {
      publish_status("ready", "急停解除，等待网页指令。");
    }
  }

  void on_goal(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    std::ostringstream detail;
    detail << "演示模式收到目标点 frame=" << msg->header.frame_id << " x=" << msg->pose.position.x
           << " y=" << msg->pose.position.y << " yaw 将在后续导航桥接节点中处理。";
    publish_status("goal_received_demo_only", detail.str());
  }

  void on_timer()
  {
    if (estop_active_) {
      return;
    }

    if (last_cmd_time_.nanoseconds() == 0) {
      return;
    }

    const double timeout_sec = get_parameter("command_timeout_sec").as_double();
    const double age_sec = (now() - last_cmd_time_).seconds();
    if (age_sec > timeout_sec && !zero_sent_after_timeout_) {
      publish_zero(
        "idle",
        output_cmd_pub_ ? "速度指令超时，已发布零速度到底盘话题。"
                        : "速度指令超时，已发布零速度到演示话题。");
      zero_sent_after_timeout_ = true;
    }
  }

  void publish_zero(const std::string & state, const std::string & detail)
  {
    geometry_msgs::msg::Twist zero;
    accepted_cmd_pub_->publish(zero);
    if (output_cmd_pub_) {
      output_cmd_pub_->publish(zero);
    }
    publish_status(state, detail);
  }

  void publish_status(const std::string & state, const std::string & detail)
  {
    std_msgs::msg::String msg;
    std::ostringstream json;
    json << "{";
    json << "\"state\":\"" << json_escape(state) << "\",";
    json << "\"detail\":\"" << json_escape(detail) << "\",";
    json << "\"estop_active\":" << (estop_active_ ? "true" : "false") << ",";
    json << "\"passthrough_enabled\":"
         << (get_parameter("enable_passthrough").as_bool() ? "true" : "false") << ",";
    json << "\"input_cmd_topic\":\"" << json_escape(input_cmd_topic_) << "\",";
    json << "\"accepted_cmd_topic\":\"" << json_escape(accepted_cmd_topic_) << "\",";
    json << "\"output_cmd_topic\":\"" << json_escape(output_cmd_topic_) << "\"";
    json << "}";
    msg.data = json.str();
    status_pub_->publish(msg);
  }

  std::string input_cmd_topic_;
  std::string estop_topic_;
  std::string goal_topic_;
  std::string accepted_cmd_topic_;
  std::string control_status_topic_;
  std::string output_cmd_topic_;
  bool estop_active_{false};
  bool zero_sent_after_timeout_{true};
  rclcpp::Time last_cmd_time_{0, 0, RCL_ROS_TIME};

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr accepted_cmd_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr output_cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CommandGuardNode>());
  rclcpp::shutdown();
  return 0;
}
