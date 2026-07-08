#include <array>
#include <algorithm>
#include <cstdio>
#include <chrono>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace
{

std::string run_command(const std::string & command)
{
  std::array<char, 256> buffer{};
  std::string output;

  FILE * pipe = popen((command + " 2>&1").c_str(), "r");
  if (pipe == nullptr) {
    return "";
  }

  while (fgets(buffer.data(), static_cast<int>(buffer.size()), pipe) != nullptr) {
    output += buffer.data();
  }
  pclose(pipe);
  return output;
}

bool contains(const std::string & text, const std::string & needle)
{
  return text.find(needle) != std::string::npos;
}

std::string json_escape(const std::string & input)
{
  std::ostringstream out;
  for (const char ch : input) {
    switch (ch) {
      case '\\':
        out << "\\\\";
        break;
      case '"':
        out << "\\\"";
        break;
      case '\n':
        out << "\\n";
        break;
      case '\r':
        break;
      case '\t':
        out << "\\t";
        break;
      default:
        out << ch;
        break;
    }
  }
  return out.str();
}

std::string bool_text(const bool value)
{
  return value ? "true" : "false";
}

std::string first_ipv4_from_interface(const std::string & interface_name)
{
  const std::string output = run_command("ip -4 addr show dev " + interface_name);
  const std::string marker = "inet ";
  const auto start = output.find(marker);
  if (start == std::string::npos) {
    return "";
  }

  const auto ip_start = start + marker.size();
  const auto slash = output.find('/', ip_start);
  if (slash == std::string::npos || slash <= ip_start) {
    return "";
  }
  return output.substr(ip_start, slash - ip_start);
}

}  // namespace

class NetworkStatusNode : public rclcpp::Node
{
public:
  NetworkStatusNode()
  : Node("network_status_node")
  {
    declare_parameter<std::string>("status_topic", "/web/network_status");
    declare_parameter<double>("check_period_sec", 5.0);
    declare_parameter<std::string>("network_mode", "same_wifi");
    declare_parameter<std::string>("wifi_interface", "wlP1p1s0");
    declare_parameter<std::string>("bind_address", "");
    declare_parameter<std::string>("advertise_address", "");
    declare_parameter<int>("http_port", 8080);
    declare_parameter<int>("api_port", 8082);

    const auto status_topic = get_parameter("status_topic").as_string();
    status_pub_ = create_publisher<std_msgs::msg::String>(
      status_topic, rclcpp::QoS(1).transient_local().reliable());

    const double period_sec = std::max(1.0, get_parameter("check_period_sec").as_double());
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(period_sec)),
      std::bind(&NetworkStatusNode::publish_status, this));

    RCLCPP_INFO(get_logger(), "network_status_node started. status_topic=%s", status_topic.c_str());
    publish_status();
  }

private:
  void publish_status()
  {
    const std::string nmcli_output = run_command("nmcli device status");
    const bool wifi_connected = contains(nmcli_output, "wifi") && contains(nmcli_output, "connected");

    const int http_port = get_parameter("http_port").as_int();
    const int api_port = get_parameter("api_port").as_int();
    const auto network_mode = get_parameter("network_mode").as_string();
    const auto wifi_interface = get_parameter("wifi_interface").as_string();
    const auto configured_bind = get_parameter("bind_address").as_string();
    const auto configured_advertise = get_parameter("advertise_address").as_string();
    const auto interface_ip = first_ipv4_from_interface(wifi_interface);
    const auto wifi_ip = !configured_advertise.empty() ? configured_advertise : interface_ip;
    const auto bind_text = configured_bind.empty() ? std::string("0.0.0.0") : configured_bind;
    const auto same_wifi_url =
      wifi_ip.empty() ? std::string("") : ("http://" + wifi_ip + ":" + std::to_string(api_port));
    const auto legacy_http_url =
      wifi_ip.empty() ? std::string("") : ("http://" + wifi_ip + ":" + std::to_string(http_port));
    const auto api_url =
      wifi_ip.empty() ? std::string("") : ("http://" + wifi_ip + ":" + std::to_string(api_port));

    std::string recommendation;
    if (!wifi_ip.empty() && wifi_connected) {
      recommendation =
        "当前只使用同一 WiFi 局域网通信：手机和设备连接同一个 WiFi 后，手机访问 " +
        same_wifi_url + "。C++节点同时提供网页和API；以太网接口保留给雷达，不用于网页服务绑定。";
    } else {
      recommendation =
        "当前配置为同一 WiFi 局域网方案，但没有检测到 WiFi IPv4 地址；请确认设备已连接 WiFi。";
    }

    std::ostringstream json;
    json << "{";
    json << "\"wifi_connected\":" << bool_text(wifi_connected) << ",";
    json << "\"network_mode\":\"" << json_escape(network_mode) << "\",";
    json << "\"wifi_interface\":\"" << json_escape(wifi_interface) << "\",";
    json << "\"wifi_ip\":\"" << json_escape(wifi_ip) << "\",";
    json << "\"bind_address\":\"" << json_escape(bind_text) << "\",";
    json << "\"same_wifi_url\":\"" << json_escape(same_wifi_url) << "\",";
    json << "\"legacy_http_url\":\"" << json_escape(legacy_http_url) << "\",";
    json << "\"api_url\":\"" << json_escape(api_url) << "\",";
    json << "\"http_port\":" << http_port << ",";
    json << "\"api_port\":" << api_port << ",";
    json << "\"recommendation\":\"" << json_escape(recommendation) << "\",";
    json << "\"nmcli_device_status\":\"" << json_escape(nmcli_output) << "\"";
    json << "}";

    std_msgs::msg::String msg;
    msg.data = json.str();
    status_pub_->publish(msg);
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NetworkStatusNode>());
  rclcpp::shutdown();
  return 0;
}
