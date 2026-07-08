#include <memory>
#include <sstream>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class DemoStateNode : public rclcpp::Node
{
public:
  DemoStateNode()
  : Node("demo_state_node"), start_time_(now())
  {
    declare_parameter<std::string>("system_status_topic", "/web/system_status");
    declare_parameter<std::string>("accepted_cmd_topic", "/web/accepted_cmd_vel");
    declare_parameter<bool>("enable_passthrough", false);
    declare_parameter<std::string>("output_cmd_topic", "/cmd_vel");
    declare_parameter<double>("publish_period_sec", 1.0);

    const auto system_status_topic = get_parameter("system_status_topic").as_string();
    const auto accepted_cmd_topic = get_parameter("accepted_cmd_topic").as_string();

    status_pub_ = create_publisher<std_msgs::msg::String>(
      system_status_topic, rclcpp::QoS(1).transient_local().reliable());
    accepted_cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      accepted_cmd_topic, 10, std::bind(&DemoStateNode::on_cmd, this, std::placeholders::_1));

    const double period_sec = std::max(0.2, get_parameter("publish_period_sec").as_double());
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(period_sec)),
      std::bind(&DemoStateNode::publish_status, this));

    RCLCPP_INFO(
      get_logger(), "demo_state_node started. status_topic=%s accepted_cmd=%s",
      system_status_topic.c_str(), accepted_cmd_topic.c_str());
    publish_status();
  }

private:
  void on_cmd(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    latest_cmd_ = *msg;
    has_cmd_ = true;
  }

  void publish_status()
  {
    std_msgs::msg::String msg;
    const double uptime_sec = (now() - start_time_).seconds();

    std::ostringstream json;
    const bool passthrough = get_parameter("enable_passthrough").as_bool();
    const auto output_topic = get_parameter("output_cmd_topic").as_string();
    json << "{";
    json << "\"mode\":\"" << (passthrough ? "vehicle_control" : "demo") << "\",";
    json << "\"message\":\""
         << (passthrough ? "网页控制链路在线，速度指令会转发到底盘话题。"
                         : "网页控制链路在线，当前不会直接控制真实底盘。")
         << "\",";
    json << "\"output_cmd_topic\":\"" << output_topic << "\",";
    json << "\"uptime_sec\":" << uptime_sec << ",";
    json << "\"has_accepted_cmd\":" << (has_cmd_ ? "true" : "false") << ",";
    json << "\"latest_cmd\":{";
    json << "\"linear_x\":" << latest_cmd_.linear.x << ",";
    json << "\"linear_y\":" << latest_cmd_.linear.y << ",";
    json << "\"angular_z\":" << latest_cmd_.angular.z;
    json << "}";
    json << "}";

    msg.data = json.str();
    status_pub_->publish(msg);
  }

  rclcpp::Time start_time_;
  geometry_msgs::msg::Twist latest_cmd_;
  bool has_cmd_{false};
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr accepted_cmd_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DemoStateNode>());
  rclcpp::shutdown();
  return 0;
}
