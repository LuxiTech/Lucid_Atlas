#include <atomic>
#include <algorithm>
#include <array>
#include <cerrno>
#include <csignal>
#include <cstdint>
#include <cmath>
#include <cstring>
#include <cstdlib>
#include <cctype>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <mutex>
#include <random>
#include <regex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <dirent.h>
#include <unistd.h>

#include <boost/asio/ip/tcp.hpp>
#include <boost/beast/core.hpp>
#include <boost/beast/http.hpp>
#include <boost/beast/version.hpp>
#include <fast_lio/srv/save_map.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rcl_interfaces/msg/parameter.hpp"
#include "rcl_interfaces/msg/parameter_type.hpp"
#include "rcl_interfaces/msg/parameter_value.hpp"
#include "rcl_interfaces/srv/get_parameters.hpp"
#include "rcl_interfaces/srv/set_parameters.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"

namespace asio = boost::asio;
namespace beast = boost::beast;
namespace http = beast::http;
using tcp = asio::ip::tcp;
namespace fs = std::filesystem;

namespace
{

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

double extract_number(const std::string & body, const std::string & key, const double fallback)
{
  const auto key_pos = body.find("\"" + key + "\"");
  if (key_pos == std::string::npos) {
    return fallback;
  }
  const auto colon = body.find(':', key_pos);
  if (colon == std::string::npos) {
    return fallback;
  }
  const auto start = body.find_first_of("-0123456789.", colon + 1);
  if (start == std::string::npos) {
    return fallback;
  }
  const auto end = body.find_first_not_of("-0123456789.eE+", start);
  try {
    return std::stod(body.substr(start, end - start));
  } catch (...) {
    return fallback;
  }
}

bool extract_bool(const std::string & body, const std::string & key, const bool fallback)
{
  const auto key_pos = body.find("\"" + key + "\"");
  if (key_pos == std::string::npos) {
    return fallback;
  }
  const auto colon = body.find(':', key_pos);
  if (colon == std::string::npos) {
    return fallback;
  }
  const auto value_start = body.find_first_not_of(" \t\n\r", colon + 1);
  if (value_start == std::string::npos) {
    return fallback;
  }
  if (body.compare(value_start, 4, "true") == 0) {
    return true;
  }
  if (body.compare(value_start, 5, "false") == 0) {
    return false;
  }
  return fallback;
}

std::string extract_string(
  const std::string & body, const std::string & key, const std::string & fallback)
{
  const auto key_pos = body.find("\"" + key + "\"");
  if (key_pos == std::string::npos) {
    return fallback;
  }
  const auto colon = body.find(':', key_pos);
  if (colon == std::string::npos) {
    return fallback;
  }
  const auto quote = body.find('"', colon + 1);
  if (quote == std::string::npos) {
    return fallback;
  }

  std::string value;
  bool escaped = false;
  for (size_t i = quote + 1; i < body.size(); ++i) {
    const char ch = body[i];
    if (escaped) {
      if (ch == 'n') {
        value.push_back('\n');
      } else {
        value.push_back(ch);
      }
      escaped = false;
      continue;
    }
    if (ch == '\\') {
      escaped = true;
      continue;
    }
    if (ch == '"') {
      return value;
    }
    value.push_back(ch);
  }
  return fallback;
}

std::string path_without_query(const std::string & target)
{
  const auto query_pos = target.find('?');
  if (query_pos == std::string::npos) {
    return target;
  }
  return target.substr(0, query_pos);
}

int hex_value(const char ch)
{
  if (ch >= '0' && ch <= '9') {
    return ch - '0';
  }
  if (ch >= 'a' && ch <= 'f') {
    return ch - 'a' + 10;
  }
  if (ch >= 'A' && ch <= 'F') {
    return ch - 'A' + 10;
  }
  return -1;
}

std::string url_decode_path(const std::string & value)
{
  std::string out;
  out.reserve(value.size());
  for (size_t i = 0; i < value.size(); ++i) {
    if (value[i] == '%' && i + 2 < value.size()) {
      const int hi = hex_value(value[i + 1]);
      const int lo = hex_value(value[i + 2]);
      if (hi >= 0 && lo >= 0) {
        out.push_back(static_cast<char>((hi << 4) | lo));
        i += 2;
        continue;
      }
    }
    out.push_back(value[i]);
  }
  return out;
}

std::string query_param(const std::string & target, const std::string & key)
{
  const auto query_pos = target.find('?');
  if (query_pos == std::string::npos) {
    return "";
  }
  const std::string query = target.substr(query_pos + 1);
  const std::string prefix = key + "=";
  size_t start = 0;
  while (start < query.size()) {
    const auto end = query.find('&', start);
    const std::string part = query.substr(start, end == std::string::npos ? end : end - start);
    if (part.rfind(prefix, 0) == 0) {
      return part.substr(prefix.size());
    }
    if (end == std::string::npos) {
      break;
    }
    start = end + 1;
  }
  return "";
}

std::string shell_quote(const std::string & value)
{
  std::string out = "'";
  for (const char ch : value) {
    if (ch == '\'') {
      out += "'\\''";
    } else {
      out.push_back(ch);
    }
  }
  out += "'";
  return out;
}

std::string sanitize_map_stem(const std::string & value)
{
  std::string clean;
  for (const char ch : value) {
    if ((ch >= 'a' && ch <= 'z') || (ch >= 'A' && ch <= 'Z') ||
      (ch >= '0' && ch <= '9') || ch == '_' || ch == '-' || ch == '.')
    {
      clean.push_back(ch);
    } else {
      clean.push_back('_');
    }
  }
  if (clean.size() >= 4 && clean.substr(clean.size() - 4) == ".pcd") {
    clean.resize(clean.size() - 4);
  }
  return clean;
}

double yaw_from_quaternion(
  const double x, const double y, const double z, const double w)
{
  const double siny_cosp = 2.0 * (w * z + x * y);
  const double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
  return std::atan2(siny_cosp, cosy_cosp);
}

}  // namespace

class WebApiNode : public rclcpp::Node
{
public:
  WebApiNode()
  : Node("web_api_node")
  {
    declare_parameter<std::string>("bind_address", "0.0.0.0");
    declare_parameter<int>("api_port", 8082);
    declare_parameter<std::string>("input_cmd_topic", "/web/cmd_vel");
    declare_parameter<std::string>("estop_topic", "/web/estop");
    declare_parameter<std::string>("goal_topic", "/web/goal_pose");
    declare_parameter<std::string>("rgb_image_topic", "/hikrobot/cu013/rgb_img");
    declare_parameter<std::string>("compressed_image_topic", "/hikrobot/cu013/rgb_img/compressed");
    declare_parameter<std::string>("camera_parameter_node", "/hikrobot/cu013/hikrobot_cu013_camera_node");
    declare_parameter<int>("jpeg_quality", 80);
    declare_parameter<std::string>("web_root", "");
    declare_parameter<std::string>("map_topic", "/map");
    declare_parameter<std::string>("static_cloud_topic", "/luxi_localization/map_cloud_floor");
    declare_parameter<std::string>("live_cloud_topic", "/luxi_localization/aligned_cloud_floor");
    declare_parameter<std::string>("static_cloud_3d_topic", "/luxi_localization/map_cloud");
    declare_parameter<std::string>("live_cloud_3d_topic", "/luxi_localization/aligned_cloud");
    declare_parameter<std::string>("mapping_cloud_topic", "/Laser_map");
    declare_parameter<bool>("restart_fast_lio_on_mapping_start", true);
    declare_parameter<std::string>("fast_lio_process_match", "fastlio_mapping");
    declare_parameter<std::string>("map_metadata_path", "");
    declare_parameter<bool>("project_mapping_cloud_to_floor", true);
    declare_parameter<bool>("project_saved_pcd_to_floor", true);
    declare_parameter<bool>("auto_fit_floor_plane", true);
    declare_parameter<int>("floor_plane_fit_iterations", 180);
    declare_parameter<int>("floor_plane_fit_max_points", 12000);
    declare_parameter<double>("floor_plane_fit_distance_threshold", 0.08);
    declare_parameter<double>("floor_plane_fit_min_inlier_ratio", 0.02);
    declare_parameter<double>("floor_plane_fit_min_abs_normal_z", 0.45);
    declare_parameter<double>("floor_plane_fit_height_band", 0.18);
    declare_parameter<double>("floor_plane_fit_min_height_bin_ratio", 0.01);
    declare_parameter<double>("floor_plane_fit_min_peak_ratio", 0.25);
    declare_parameter<std::vector<double>>(
      "floor_plane", {-0.357093, 0.006594, 0.934045, 1.001688});
    declare_parameter<std::string>("pose_topic", "/localization_2d");
    declare_parameter<std::string>("odom_topic", "/Odometry");
    declare_parameter<std::string>("initial_pose_topic", "/initialpose");
    declare_parameter<std::string>("localization_status_topic", "/luxi_localization/status");
    declare_parameter<std::string>("nav_path_topic", "/nav/path");
    declare_parameter<std::string>("nav_local_path_topic", "/nav/local_path");
    declare_parameter<std::string>("nav_costmap_topic", "/nav/costmap");
    declare_parameter<std::string>("nav_cmd_topic", "/nav/cmd_vel");
    declare_parameter<std::string>("nav_stop_topic", "/nav/stop");
    declare_parameter<std::string>("nav_status_topic", "/nav/status");
    declare_parameter<std::string>("nav_parameter_node", "/luxi_navigation_node");
    declare_parameter<std::string>("map_save_service", "/map_save_with_name");
    declare_parameter<std::string>("saved_maps_dir",
      "/home/nvidia/project/luxi-atlas/project/luxi-atlas-lio/maps");
    declare_parameter<std::string>("pcd_to_pgm_binary",
      "/home/nvidia/project/luxi-atlas/project/luxi-atlas-lio/tools/transfrom_tools/pcd_to_pgm/build/pcd2pgm");
    declare_parameter<std::string>("pcd_to_pgm_output_dir",
      "/home/nvidia/project/luxi-atlas/project/luxi-atlas-lio/tools/transfrom_tools/pcd_to_pgm/output");
    declare_parameter<double>("converted_map_min_z", 0.50);
    declare_parameter<double>("converted_map_max_z", 2.00);
    declare_parameter<int>("converted_map_min_component_cells", 30);
    declare_parameter<double>("converted_map_auto_floor_target_obstacle_ratio", 0.18);
    declare_parameter<double>("converted_map_auto_floor_max_obstacle_ratio", 0.35);
    declare_parameter<std::string>("converted_map_suffix", "_nav_floor_plane_autofit_h050_200_clean30");
    declare_parameter<int>("max_grid_cells", 250000);
    declare_parameter<int>("max_static_cloud_points", 60000);
    declare_parameter<int>("max_live_cloud_points", 12000);
    declare_parameter<int>("max_mapping_cloud_points", 20000);
    declare_parameter<std::string>("system_status_topic", "/web/system_status");
    declare_parameter<std::string>("control_status_topic", "/web/control_status");
    declare_parameter<std::string>("network_status_topic", "/web/network_status");

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(
      get_parameter("input_cmd_topic").as_string(), 10);
    estop_pub_ = create_publisher<std_msgs::msg::Bool>(
      get_parameter("estop_topic").as_string(), 10);
    goal_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      get_parameter("goal_topic").as_string(), 10);
    initial_pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      get_parameter("initial_pose_topic").as_string(), 10);
    nav_stop_pub_ = create_publisher<std_msgs::msg::Bool>(
      get_parameter("nav_stop_topic").as_string(), 10);
    const auto camera_parameter_node = get_parameter("camera_parameter_node").as_string();
    camera_set_parameters_client_ = create_client<rcl_interfaces::srv::SetParameters>(
      camera_parameter_node + "/set_parameters");
    const auto nav_parameter_node = get_parameter("nav_parameter_node").as_string();
    nav_get_parameters_client_ = create_client<rcl_interfaces::srv::GetParameters>(
      nav_parameter_node + "/get_parameters");
    nav_set_parameters_client_ = create_client<rcl_interfaces::srv::SetParameters>(
      nav_parameter_node + "/set_parameters");
    map_save_client_ = create_client<fast_lio::srv::SaveMap>(
      get_parameter("map_save_service").as_string());
    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      get_parameter("rgb_image_topic").as_string(), rclcpp::SensorDataQoS(),
      std::bind(&WebApiNode::on_image, this, std::placeholders::_1));
    compressed_image_sub_ = create_subscription<sensor_msgs::msg::CompressedImage>(
      get_parameter("compressed_image_topic").as_string(), rclcpp::SensorDataQoS(),
      std::bind(&WebApiNode::on_compressed_image, this, std::placeholders::_1));
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      get_parameter("map_topic").as_string(), rclcpp::QoS(1).transient_local().reliable(),
      std::bind(&WebApiNode::on_map, this, std::placeholders::_1));
    static_cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      get_parameter("static_cloud_topic").as_string(), rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        on_cloud(msg, true);
      });
    live_cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      get_parameter("live_cloud_topic").as_string(), rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        on_cloud(msg, false);
      });
    static_cloud_3d_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      get_parameter("static_cloud_3d_topic").as_string(), rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        on_cloud_3d(msg, true);
      });
    live_cloud_3d_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      get_parameter("live_cloud_3d_topic").as_string(), rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        on_cloud_3d(msg, false);
      });
    mapping_cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      get_parameter("mapping_cloud_topic").as_string(), rclcpp::QoS(1).reliable(),
      std::bind(&WebApiNode::on_mapping_cloud, this, std::placeholders::_1));
    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      get_parameter("pose_topic").as_string(), 10,
      std::bind(&WebApiNode::on_pose, this, std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      get_parameter("odom_topic").as_string(), 10,
      std::bind(&WebApiNode::on_odom, this, std::placeholders::_1));
    localization_status_sub_ = create_subscription<std_msgs::msg::String>(
      get_parameter("localization_status_topic").as_string(),
      rclcpp::QoS(1).transient_local().reliable(),
      [this](std_msgs::msg::String::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(map_mutex_);
        localization_status_ = msg->data;
      });
    nav_path_sub_ = create_subscription<nav_msgs::msg::Path>(
      get_parameter("nav_path_topic").as_string(), rclcpp::QoS(1).transient_local().reliable(),
      std::bind(&WebApiNode::on_nav_path, this, std::placeholders::_1));
    nav_local_path_sub_ = create_subscription<nav_msgs::msg::Path>(
      get_parameter("nav_local_path_topic").as_string(), rclcpp::QoS(1).transient_local().reliable(),
      std::bind(&WebApiNode::on_nav_local_path, this, std::placeholders::_1));
    nav_costmap_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      get_parameter("nav_costmap_topic").as_string(), rclcpp::QoS(1).transient_local().reliable(),
      std::bind(&WebApiNode::on_nav_costmap, this, std::placeholders::_1));
    nav_cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      get_parameter("nav_cmd_topic").as_string(), 10,
      std::bind(&WebApiNode::on_nav_cmd, this, std::placeholders::_1));
    nav_status_sub_ = create_subscription<std_msgs::msg::String>(
      get_parameter("nav_status_topic").as_string(), rclcpp::QoS(1).transient_local().reliable(),
      [this](std_msgs::msg::String::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(map_mutex_);
        nav_status_ = msg->data;
      });

    system_sub_ = create_subscription<std_msgs::msg::String>(
      get_parameter("system_status_topic").as_string(), rclcpp::QoS(1).transient_local().reliable(),
      [this](std_msgs::msg::String::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(status_mutex_);
        system_status_ = msg->data;
      });
    control_sub_ = create_subscription<std_msgs::msg::String>(
      get_parameter("control_status_topic").as_string(), rclcpp::QoS(1).transient_local().reliable(),
      [this](std_msgs::msg::String::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(status_mutex_);
        control_status_ = msg->data;
      });
    network_sub_ = create_subscription<std_msgs::msg::String>(
      get_parameter("network_status_topic").as_string(), rclcpp::QoS(1).transient_local().reliable(),
      [this](std_msgs::msg::String::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(status_mutex_);
        network_status_ = msg->data;
      });

    running_.store(true);
    server_thread_ = std::thread([this]() { run_server(); });

    RCLCPP_INFO(
      get_logger(), "web_api_node started. http://%s:%ld/ and /api/status",
      get_parameter("bind_address").as_string().c_str(),
      static_cast<long>(get_parameter("api_port").as_int()));
  }

  ~WebApiNode() override
  {
    running_.store(false);
    try {
      if (acceptor_) {
        beast::error_code ec;
        acceptor_->close(ec);
      }
      io_context_.stop();
    } catch (...) {
    }
    if (server_thread_.joinable()) {
      server_thread_.join();
    }
  }

private:
  void run_server()
  {
    try {
      const auto bind_address = asio::ip::make_address(get_parameter("bind_address").as_string());
      const auto port = static_cast<unsigned short>(get_parameter("api_port").as_int());
      acceptor_ = std::make_unique<tcp::acceptor>(io_context_, tcp::endpoint(bind_address, port));
      acceptor_->non_blocking(true);

      while (running_.load()) {
        beast::error_code ec;
        tcp::socket socket(io_context_);
        acceptor_->accept(socket, ec);
        if (ec) {
          if (ec == asio::error::would_block || ec == asio::error::try_again) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
          }
          if (running_.load()) {
            RCLCPP_WARN(get_logger(), "HTTP API accept failed: %s", ec.message().c_str());
          }
          continue;
        }
        std::thread([this](tcp::socket session_socket) {
          handle_session(std::move(session_socket));
        }, std::move(socket)).detach();
      }
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(get_logger(), "HTTP API server stopped: %s", ex.what());
    }
  }

  http::response<http::string_body> make_json_response(
    const http::request<http::string_body> & req, const http::status status,
    const std::string & body)
  {
    http::response<http::string_body> res{status, req.version()};
    res.set(http::field::server, "luxi_web_control");
    res.set(http::field::content_type, "application/json; charset=utf-8");
    res.set(http::field::cache_control, "no-store, no-cache, max-age=0");
    res.set(http::field::access_control_allow_origin, "*");
    res.set(http::field::access_control_allow_methods, "GET, POST, OPTIONS");
    res.set(http::field::access_control_allow_headers, "content-type, cache-control");
    res.keep_alive(false);
    res.body() = body;
    res.prepare_payload();
    return res;
  }

  http::response<http::string_body> make_jpeg_response(
    const http::request<http::string_body> & req, const std::string & jpeg)
  {
    http::response<http::string_body> res{http::status::ok, req.version()};
    res.set(http::field::server, "luxi_web_control");
    res.set(http::field::content_type, "image/jpeg");
    res.set(http::field::cache_control, "no-store, no-cache, max-age=0");
    res.set(http::field::access_control_allow_origin, "*");
    res.keep_alive(false);
    res.body() = jpeg;
    res.prepare_payload();
    return res;
  }

  http::response<http::string_body> make_binary_response(
    const http::request<http::string_body> & req,
    const std::string & content_type,
    const std::string & body)
  {
    http::response<http::string_body> res{http::status::ok, req.version()};
    res.set(http::field::server, "luxi_web_control");
    res.set(http::field::content_type, content_type);
    res.set(http::field::cache_control, "no-store, no-cache, max-age=0");
    res.set(http::field::access_control_allow_origin, "*");
    res.keep_alive(false);
    res.body() = body;
    res.prepare_payload();
    return res;
  }

  std::string static_content_type(const fs::path & path) const
  {
    const std::string ext = path.extension().string();
    if (ext == ".html" || ext == ".htm") {
      return "text/html; charset=utf-8";
    }
    if (ext == ".js" || ext == ".mjs") {
      return "text/javascript; charset=utf-8";
    }
    if (ext == ".css") {
      return "text/css; charset=utf-8";
    }
    if (ext == ".json") {
      return "application/json; charset=utf-8";
    }
    if (ext == ".png") {
      return "image/png";
    }
    if (ext == ".jpg" || ext == ".jpeg") {
      return "image/jpeg";
    }
    if (ext == ".svg") {
      return "image/svg+xml";
    }
    if (ext == ".ico") {
      return "image/x-icon";
    }
    if (ext == ".wasm") {
      return "application/wasm";
    }
    return "application/octet-stream";
  }

  bool resolve_static_path(const std::string & target, fs::path & file_path, std::string & error) const
  {
    const auto web_root_param = get_parameter("web_root").as_string();
    if (web_root_param.empty()) {
      error = "web_root is not configured";
      return false;
    }

    const fs::path web_root = fs::absolute(fs::path(web_root_param)).lexically_normal();
    std::string decoded = url_decode_path(target);
    if (decoded.empty() || decoded == "/") {
      decoded = "/index.html";
    }
    if (decoded.find('\0') != std::string::npos) {
      error = "invalid static file path";
      return false;
    }
    if (!decoded.empty() && decoded.front() == '/') {
      decoded.erase(decoded.begin());
    }

    fs::path relative;
    for (const auto & part : fs::path(decoded)) {
      const std::string part_text = part.string();
      if (part_text.empty() || part_text == ".") {
        continue;
      }
      if (part_text == "..") {
        error = "path traversal is not allowed";
        return false;
      }
      relative /= part;
    }

    fs::path candidate = fs::absolute(web_root / relative).lexically_normal();
    const auto root_text = web_root.string();
    const auto candidate_text = candidate.string();
    if (candidate_text != root_text &&
      candidate_text.rfind(root_text + fs::path::preferred_separator, 0) != 0)
    {
      error = "static file path is outside web_root";
      return false;
    }
    if (!fs::exists(candidate) || !fs::is_regular_file(candidate)) {
      error = "static file not found";
      return false;
    }
    file_path = candidate;
    return true;
  }

  http::response<http::string_body> make_static_file_response(
    const http::request<http::string_body> & req,
    const fs::path & file_path)
  {
    std::ifstream input(file_path, std::ios::binary);
    if (!input) {
      return make_json_response(
        req, http::status::service_unavailable,
        "{\"ok\":false,\"error\":\"failed to read static file\"}");
    }
    std::ostringstream body;
    body << input.rdbuf();

    http::response<http::string_body> res{http::status::ok, req.version()};
    res.set(http::field::server, "luxi_web_control");
    res.set(http::field::content_type, static_content_type(file_path));
    res.set(http::field::cache_control, "no-store, no-cache, max-age=0");
    res.set(http::field::access_control_allow_origin, "*");
    res.keep_alive(false);
    if (req.method() != http::verb::head) {
      res.body() = body.str();
    }
    res.prepare_payload();
    return res;
  }

  http::response<http::string_body> handle_request(
    const http::request<http::string_body> & req)
  {
    const std::string target = path_without_query(std::string(req.target()));
    if (req.method() == http::verb::options) {
      return make_json_response(req, http::status::ok, "{\"ok\":true}");
    }

    if (req.method() == http::verb::get && target == "/api/camera/latest.jpg") {
      std::lock_guard<std::mutex> lock(image_mutex_);
      if (latest_jpeg_.empty()) {
        return make_json_response(
          req, http::status::service_unavailable,
          "{\"ok\":false,\"error\":\"no camera frame yet\"}");
      }
      return make_jpeg_response(req, latest_jpeg_);
    }

    if (req.method() == http::verb::get && target == "/api/map/status") {
      return make_json_response(req, http::status::ok, make_map_status_json());
    }

    if (req.method() == http::verb::get && target == "/api/map/snapshot") {
      return make_json_response(req, http::status::ok, make_map_snapshot_json());
    }

    if (req.method() == http::verb::get && target == "/api/mapping/status") {
      return make_json_response(req, http::status::ok, make_mapping_status_json());
    }

    if (req.method() == http::verb::get && target == "/api/maps") {
      return make_json_response(req, http::status::ok, make_saved_maps_json());
    }

    if (req.method() == http::verb::post && target == "/api/maps/convert") {
      const std::string map_name = extract_string(req.body(), "map_name", "");
      std::string response_json;
      const bool ok = convert_saved_map(map_name, response_json);
      return make_json_response(
        req, ok ? http::status::ok : http::status::service_unavailable, response_json);
    }

    if (req.method() == http::verb::get && target == "/api/maps/view") {
      const std::string map_name = query_param(std::string(req.target()), "map_name");
      std::string response_json;
      const bool ok = make_saved_map_view_json(map_name, response_json);
      return make_json_response(
        req, ok ? http::status::ok : http::status::service_unavailable, response_json);
    }

    if (req.method() == http::verb::get && target == "/api/maps/preview.png") {
      const std::string map_name = query_param(std::string(req.target()), "map_name");
      std::string png;
      std::string error;
      if (!make_saved_map_preview_png(map_name, png, error)) {
        return make_json_response(
          req, http::status::not_found,
          "{\"ok\":false,\"error\":\"" + json_escape(error) + "\"}");
      }
      return make_binary_response(req, "image/png", png);
    }

    if (req.method() == http::verb::get && target == "/api/status") {
      std::lock_guard<std::mutex> lock(status_mutex_);
      const bool has_camera_frame = has_camera_frame_.load();
      std::ostringstream json;
      json << "{";
      json << "\"ok\":true,";
      json << "\"has_camera_frame\":" << (has_camera_frame ? "true" : "false") << ",";
      json << "\"rgb_image_topic\":\"" << json_escape(get_parameter("rgb_image_topic").as_string()) << "\",";
      json << "\"compressed_image_topic\":\"" << json_escape(get_parameter("compressed_image_topic").as_string()) << "\",";
      json << "\"camera_source\":\"" << json_escape(latest_image_source_) << "\",";
      json << "\"camera_parameter_node\":\"" << json_escape(get_parameter("camera_parameter_node").as_string()) << "\",";
      json << "\"map_topic\":\"" << json_escape(get_parameter("map_topic").as_string()) << "\",";
      json << "\"static_cloud_topic\":\"" << json_escape(get_parameter("static_cloud_topic").as_string()) << "\",";
      json << "\"live_cloud_topic\":\"" << json_escape(get_parameter("live_cloud_topic").as_string()) << "\",";
      json << "\"mapping_cloud_topic\":\"" << json_escape(get_parameter("mapping_cloud_topic").as_string()) << "\",";
      json << "\"pose_topic\":\"" << json_escape(get_parameter("pose_topic").as_string()) << "\",";
      json << "\"odom_topic\":\"" << json_escape(get_parameter("odom_topic").as_string()) << "\",";
      json << "\"initial_pose_topic\":\"" << json_escape(get_parameter("initial_pose_topic").as_string()) << "\",";
      json << "\"localization_status_topic\":\"" <<
        json_escape(get_parameter("localization_status_topic").as_string()) << "\",";
      json << "\"nav_path_topic\":\"" << json_escape(get_parameter("nav_path_topic").as_string()) << "\",";
      json << "\"nav_local_path_topic\":\"" <<
        json_escape(get_parameter("nav_local_path_topic").as_string()) << "\",";
      json << "\"nav_costmap_topic\":\"" << json_escape(get_parameter("nav_costmap_topic").as_string()) <<
        "\",";
      json << "\"nav_status_topic\":\"" << json_escape(get_parameter("nav_status_topic").as_string()) << "\",";
      json << "\"map_save_service\":\"" << json_escape(get_parameter("map_save_service").as_string()) << "\",";
      json << "\"camera_publishers\":" <<
        count_publishers(get_parameter("compressed_image_topic").as_string()) << ",";
      json << "\"raw_camera_publishers\":" <<
        count_publishers(get_parameter("rgb_image_topic").as_string()) << ",";
      json << "\"map_publishers\":" <<
        count_publishers(get_parameter("map_topic").as_string()) << ",";
      json << "\"static_cloud_publishers\":" <<
        count_publishers(get_parameter("static_cloud_topic").as_string()) << ",";
      json << "\"live_cloud_publishers\":" <<
        count_publishers(get_parameter("live_cloud_topic").as_string()) << ",";
      json << "\"mapping_cloud_publishers\":" <<
        count_publishers(get_parameter("mapping_cloud_topic").as_string()) << ",";
      json << "\"pose_publishers\":" <<
        count_publishers(get_parameter("pose_topic").as_string()) << ",";
      json << "\"nav_path_publishers\":" <<
        count_publishers(get_parameter("nav_path_topic").as_string()) << ",";
      json << "\"nav_local_path_publishers\":" <<
        count_publishers(get_parameter("nav_local_path_topic").as_string()) << ",";
      json << "\"nav_costmap_publishers\":" <<
        count_publishers(get_parameter("nav_costmap_topic").as_string()) << ",";
      json << "\"system_status\":\"" << json_escape(system_status_) << "\",";
      json << "\"control_status\":\"" << json_escape(control_status_) << "\",";
      json << "\"network_status\":\"" << json_escape(network_status_) << "\",";
      json << "\"nav_status\":\"" << json_escape(nav_status_) << "\"";
      json << "}";
      return make_json_response(req, http::status::ok, json.str());
    }

    if (req.method() == http::verb::get && target == "/api/client-config") {
      const auto bind_address = get_parameter("bind_address").as_string();
      const auto api_port = get_parameter("api_port").as_int();
      std::ostringstream json;
      json << "{";
      json << "\"ok\":true,";
      json << "\"api_port\":" << api_port << ",";
      json << "\"bind_address\":\"" << json_escape(bind_address) << "\",";
      json << "\"same_origin_api\":true,";
      json << "\"android_recommended_path\":\"/\"";
      json << "}";
      return make_json_response(req, http::status::ok, json.str());
    }

    if (req.method() == http::verb::post && target == "/api/mapping/start") {
      const std::string map_name = extract_string(req.body(), "map_name", "");
      {
        std::lock_guard<std::mutex> lock(mapping_mutex_);
        mapping_active_ = false;
        mapping_start_time_valid_ = false;
      }
      clear_mapping_clouds();
      const std::string restart_message = restart_fast_lio_for_mapping();
      {
        std::lock_guard<std::mutex> lock(mapping_mutex_);
        mapping_started_at_ = now();
        mapping_start_time_valid_ = true;
        mapping_active_ = true;
        mapping_map_name_ = map_name;
        mapping_message_ =
          "建图传输已开启，网页缓存已清空；" + restart_message;
      }
      return make_json_response(req, http::status::ok, make_mapping_status_json());
    }

    if (req.method() == http::verb::post && target == "/api/mapping/stop") {
      clear_mapping_clouds();
      {
        std::lock_guard<std::mutex> lock(mapping_mutex_);
        mapping_active_ = false;
        mapping_start_time_valid_ = false;
        mapping_message_ =
          "建图传输已停止；FAST-LIO仍为定位运行，网页不再接收累计建图点云。";
      }
      return make_json_response(req, http::status::ok, make_mapping_status_json());
    }

    if (req.method() == http::verb::post && target == "/api/mapping/save") {
      const std::string request_name = extract_string(req.body(), "map_name", "");
      std::string map_name = request_name;
      if (map_name.empty()) {
        std::lock_guard<std::mutex> lock(mapping_mutex_);
        map_name = mapping_map_name_;
      }
      if (map_name.empty()) {
        return make_json_response(
          req, http::status::bad_request,
          "{\"ok\":false,\"error\":\"map_name is required\"}");
      }

      std::string response_json;
      const bool ok = save_mapping_pcd(map_name, response_json);
      return make_json_response(
        req, ok ? http::status::ok : http::status::service_unavailable, response_json);
    }

    if (req.method() == http::verb::post && target == "/api/cmd_vel") {
      const double seq_value = extract_number(req.body(), "command_seq", -1.0);
      const std::string command_session = extract_string(req.body(), "command_session", "");
      uint64_t command_seq = 0;
      if (seq_value >= 0.0 && std::isfinite(seq_value)) {
        command_seq = static_cast<uint64_t>(seq_value);
        const std::string session_key = command_session.empty() ? "__legacy__" : command_session;
        {
          std::lock_guard<std::mutex> lock(cmd_mutex_);
          if (session_key == latest_cmd_session_ && command_seq <= latest_cmd_seq_) {
            std::ostringstream stale_json;
            stale_json << "{\"ok\":true,\"stale\":true,"
                       << "\"command_session\":\"" << json_escape(command_session) << "\","
                       << "\"command_seq\":" << command_seq << ","
                       << "\"latest_command_seq\":" << latest_cmd_seq_ << "}";
            return make_json_response(req, http::status::ok, stale_json.str());
          }
          latest_cmd_session_ = session_key;
          latest_cmd_seq_ = command_seq;
        }
      }
      geometry_msgs::msg::Twist msg;
      msg.linear.x = extract_number(req.body(), "linear_x", 0.0);
      msg.linear.y = extract_number(req.body(), "linear_y", 0.0);
      msg.angular.z = extract_number(req.body(), "angular_z", 0.0);
      cmd_pub_->publish(msg);
      std::ostringstream json;
      json << std::fixed << std::setprecision(4)
           << "{\"ok\":true,\"stale\":false,\"topic\":\"/web/cmd_vel\","
           << "\"command_session\":\"" << json_escape(command_session) << "\","
           << "\"command_seq\":" << command_seq << ","
           << "\"linear_x\":" << msg.linear.x << ","
           << "\"linear_y\":" << msg.linear.y << ","
           << "\"angular_z\":" << msg.angular.z << "}";
      return make_json_response(req, http::status::ok, json.str());
    }

    if (req.method() == http::verb::post && target == "/api/nav/start") {
      nav_drive_active_.store(true);
      return make_json_response(
        req, http::status::ok,
        "{\"ok\":true,\"nav_drive_active\":true,\"input\":\"/nav/cmd_vel\",\"output\":\"/web/cmd_vel\"}");
    }

    if (req.method() == http::verb::post && target == "/api/nav/stop") {
      nav_drive_active_.store(false);
      geometry_msgs::msg::Twist zero;
      cmd_pub_->publish(zero);
      std_msgs::msg::Bool stop;
      stop.data = true;
      nav_stop_pub_->publish(stop);
      {
        std::lock_guard<std::mutex> lock(map_mutex_);
        nav_path_ = PathSnapshot{};
        nav_local_path_ = PathSnapshot{};
      }
      return make_json_response(
        req, http::status::ok,
        "{\"ok\":true,\"nav_drive_active\":false,\"output\":\"/web/cmd_vel\","
        "\"stop_topic\":\"/nav/stop\"}");
    }

    if (
      (req.method() == http::verb::get || req.method() == http::verb::post) &&
      target == "/api/nav/config")
    {
      if (req.method() == http::verb::post) {
        const double max_vel_x = std::clamp(
          extract_number(req.body(), "max_vel_x", 0.22), 0.05, 0.80);
        const double min_nonzero_vel_x = std::clamp(
          extract_number(req.body(), "min_nonzero_vel_x", std::min(0.10, max_vel_x)),
          0.0, max_vel_x);
        const double max_vel_theta = std::clamp(
          extract_number(req.body(), "max_vel_theta", 0.75), 0.10, 2.00);
        const double dwb_velocity_bias = std::clamp(
          extract_number(req.body(), "dwb_velocity_bias", 0.4), 0.0, 5.0);
        std::string error;
        if (!set_navigation_config(max_vel_x, min_nonzero_vel_x, max_vel_theta, dwb_velocity_bias, error)) {
          return make_json_response(
            req, http::status::service_unavailable,
            "{\"ok\":false,\"error\":\"" + json_escape(error) + "\"}");
        }
      }

      std::string json;
      std::string error;
      if (!get_navigation_config(json, error)) {
        return make_json_response(
          req, http::status::service_unavailable,
          "{\"ok\":false,\"error\":\"" + json_escape(error) + "\"}");
      }
      return make_json_response(req, http::status::ok, json);
    }

    if (req.method() == http::verb::post && target == "/api/estop") {
      std_msgs::msg::Bool msg;
      msg.data = extract_bool(req.body(), "active", true);
      estop_pub_->publish(msg);
      return make_json_response(req, http::status::ok, "{\"ok\":true,\"topic\":\"/web/estop\"}");
    }

    if (req.method() == http::verb::post && target == "/api/goal_pose") {
      const double x = extract_number(req.body(), "x", 0.0);
      const double y = extract_number(req.body(), "y", 0.0);
      const double yaw = extract_number(req.body(), "yaw", 0.0);

      geometry_msgs::msg::PoseStamped msg;
      msg.header.stamp = now();
      msg.header.frame_id = "map";
      msg.pose.position.x = x;
      msg.pose.position.y = y;
      msg.pose.position.z = 0.0;
      msg.pose.orientation.z = std::sin(yaw * 0.5);
      msg.pose.orientation.w = std::cos(yaw * 0.5);
      goal_pub_->publish(msg);
      return make_json_response(req, http::status::ok, "{\"ok\":true,\"topic\":\"/web/goal_pose\"}");
    }

    if (req.method() == http::verb::post && target == "/api/map/initial_pose") {
      const double x = extract_number(req.body(), "x", 0.0);
      const double y = extract_number(req.body(), "y", 0.0);
      const double yaw = extract_number(req.body(), "yaw", 0.0);

      geometry_msgs::msg::PoseWithCovarianceStamped msg;
      msg.header.stamp = now();
      msg.header.frame_id = "map";
      msg.pose.pose.position.x = x;
      msg.pose.pose.position.y = y;
      msg.pose.pose.position.z = 0.0;
      msg.pose.pose.orientation.z = std::sin(yaw * 0.5);
      msg.pose.pose.orientation.w = std::cos(yaw * 0.5);
      msg.pose.covariance[0] = 0.25;
      msg.pose.covariance[7] = 0.25;
      msg.pose.covariance[35] = 0.06853892326654787;
      initial_pose_pub_->publish(msg);
      constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;
      RCLCPP_INFO(
        get_logger(), "published initial pose x=%.3f y=%.3f yaw=%.3f rad (%.1f deg)",
        x, y, yaw, yaw * kRadToDeg);
      std::ostringstream json;
      json << "{\"ok\":true,\"topic\":\"/initialpose\","
           << "\"x\":" << x << ","
           << "\"y\":" << y << ","
           << "\"yaw\":" << yaw << ","
           << "\"yaw_deg\":" << (yaw * kRadToDeg) << "}";
      return make_json_response(req, http::status::ok, json.str());
    }

    if (req.method() == http::verb::post && target == "/api/camera/exposure") {
      const double exposure_time_us = extract_number(req.body(), "exposure_time_us", -1.0);
      if (exposure_time_us <= 0.0) {
        return make_json_response(
          req, http::status::bad_request,
          "{\"ok\":false,\"error\":\"exposure_time_us must be positive\"}");
      }

      std::string error;
      if (!set_camera_exposure(exposure_time_us, error)) {
        return make_json_response(
          req, http::status::service_unavailable,
          "{\"ok\":false,\"error\":\"" + json_escape(error) + "\"}");
      }

      std::ostringstream json;
      json << "{\"ok\":true,\"exposure_auto\":\"Off\",\"exposure_time_us\":" << exposure_time_us << "}";
      return make_json_response(req, http::status::ok, json.str());
    }

    if ((req.method() == http::verb::get || req.method() == http::verb::head) &&
      target.rfind("/api/", 0) != 0)
    {
      fs::path static_path;
      std::string error;
      if (resolve_static_path(target, static_path, error)) {
        return make_static_file_response(req, static_path);
      }
    }

    return make_json_response(req, http::status::not_found, "{\"ok\":false,\"error\":\"not found\"}");
  }

  bool set_camera_exposure(const double exposure_time_us, std::string & error)
  {
    if (!camera_set_parameters_client_) {
      error = "camera parameter client is not initialized";
      return false;
    }
    if (!camera_set_parameters_client_->wait_for_service(std::chrono::milliseconds(500))) {
      error = "camera parameter service is not available";
      return false;
    }

    auto request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
    request->parameters.push_back(make_string_parameter("exposure_auto", "Off"));
    request->parameters.push_back(make_double_parameter("exposure_time_us", exposure_time_us));

    auto future = camera_set_parameters_client_->async_send_request(request);
    const auto status = future.wait_for(std::chrono::seconds(2));
    if (status != std::future_status::ready) {
      error = "camera parameter service timed out";
      return false;
    }

    const auto response = future.get();
    for (const auto & result : response->results) {
      if (!result.successful) {
        error = result.reason.empty() ? "camera rejected parameter update" : result.reason;
        return false;
      }
    }

    return true;
  }

  bool set_navigation_config(
    const double max_vel_x,
    const double min_nonzero_vel_x,
    const double max_vel_theta,
    const double dwb_velocity_bias,
    std::string & error)
  {
    if (!nav_set_parameters_client_) {
      error = "navigation parameter client is not initialized";
      return false;
    }
    if (!nav_set_parameters_client_->wait_for_service(std::chrono::milliseconds(500))) {
      error = "navigation parameter service is not available";
      return false;
    }

    auto request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
    request->parameters.push_back(make_double_parameter("max_vel_x", max_vel_x));
    request->parameters.push_back(make_double_parameter("min_nonzero_vel_x", min_nonzero_vel_x));
    request->parameters.push_back(make_double_parameter("max_vel_theta", max_vel_theta));
    request->parameters.push_back(make_double_parameter("dwb_velocity_bias", dwb_velocity_bias));

    auto future = nav_set_parameters_client_->async_send_request(request);
    const auto status = future.wait_for(std::chrono::seconds(2));
    if (status != std::future_status::ready) {
      error = "navigation parameter service timed out";
      return false;
    }

    const auto response = future.get();
    for (const auto & result : response->results) {
      if (!result.successful) {
        error = result.reason.empty() ? "navigation rejected parameter update" : result.reason;
        return false;
      }
    }

    RCLCPP_INFO(
      get_logger(),
      "updated navigation config max_vel_x=%.3f min_nonzero_vel_x=%.3f max_vel_theta=%.3f velocity_bias=%.3f",
      max_vel_x, min_nonzero_vel_x, max_vel_theta, dwb_velocity_bias);
    return true;
  }

  bool get_navigation_config(std::string & json, std::string & error)
  {
    if (!nav_get_parameters_client_) {
      error = "navigation parameter client is not initialized";
      return false;
    }
    if (!nav_get_parameters_client_->wait_for_service(std::chrono::milliseconds(500))) {
      error = "navigation parameter service is not available";
      return false;
    }

    auto request = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
    request->names = {
      "max_vel_x",
      "min_nonzero_vel_x",
      "max_vel_theta",
      "dwb_velocity_bias"
    };

    auto future = nav_get_parameters_client_->async_send_request(request);
    const auto status = future.wait_for(std::chrono::seconds(2));
    if (status != std::future_status::ready) {
      error = "navigation parameter service timed out";
      return false;
    }

    const auto response = future.get();
    if (response->values.size() != request->names.size()) {
      error = "navigation parameter response size mismatch";
      return false;
    }

    auto double_value = [](const rcl_interfaces::msg::ParameterValue & value) {
      if (value.type == rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER) {
        return static_cast<double>(value.integer_value);
      }
      return value.double_value;
    };

    std::ostringstream out;
    out << "{\"ok\":true,"
        << "\"max_vel_x\":" << double_value(response->values[0]) << ","
        << "\"min_nonzero_vel_x\":" << double_value(response->values[1]) << ","
        << "\"max_vel_theta\":" << double_value(response->values[2]) << ","
        << "\"dwb_velocity_bias\":" << double_value(response->values[3]) << "}";
    json = out.str();
    return true;
  }

  rcl_interfaces::msg::Parameter make_string_parameter(
    const std::string & name, const std::string & value) const
  {
    rcl_interfaces::msg::Parameter parameter;
    parameter.name = name;
    parameter.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
    parameter.value.string_value = value;
    return parameter;
  }

  rcl_interfaces::msg::Parameter make_double_parameter(
    const std::string & name, const double value) const
  {
    rcl_interfaces::msg::Parameter parameter;
    parameter.name = name;
    parameter.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    parameter.value.double_value = value;
    return parameter;
  }

  void handle_session(tcp::socket socket)
  {
    beast::flat_buffer buffer;
    http::request<http::string_body> req;
    beast::error_code ec;
    http::read(socket, buffer, req, ec);
    if (ec) {
      return;
    }

    const std::string target = path_without_query(std::string(req.target()));
    if (req.method() == http::verb::get && target == "/api/camera/stream.mjpg") {
      write_mjpeg_stream(std::move(socket));
      return;
    }

    auto res = handle_request(req);
    http::write(socket, res, ec);
    socket.shutdown(tcp::socket::shutdown_send, ec);
  }

  void write_mjpeg_stream(tcp::socket socket)
  {
    beast::error_code ec;
    const std::string header =
      "HTTP/1.1 200 OK\r\n"
      "Server: luxi_web_control\r\n"
      "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n"
      "Cache-Control: no-store, no-cache, max-age=0\r\n"
      "Access-Control-Allow-Origin: *\r\n"
      "Connection: close\r\n"
      "\r\n";
    asio::write(socket, asio::buffer(header), ec);
    if (ec) {
      return;
    }

    while (running_.load()) {
      std::string jpeg;
      {
        std::lock_guard<std::mutex> lock(image_mutex_);
        jpeg = latest_jpeg_;
      }

      if (jpeg.empty()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        continue;
      }

      std::ostringstream part_header;
      part_header << "--frame\r\n";
      part_header << "Content-Type: image/jpeg\r\n";
      part_header << "Content-Length: " << jpeg.size() << "\r\n\r\n";
      const std::string part_header_text = part_header.str();
      const std::string part_tail = "\r\n";

      asio::write(socket, asio::buffer(part_header_text), ec);
      if (ec) {
        break;
      }
      asio::write(socket, asio::buffer(jpeg), ec);
      if (ec) {
        break;
      }
      asio::write(socket, asio::buffer(part_tail), ec);
      if (ec) {
        break;
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    socket.shutdown(tcp::socket::shutdown_send, ec);
  }

  void on_compressed_image(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
  {
    if (msg->data.empty()) {
      return;
    }

    std::lock_guard<std::mutex> lock(image_mutex_);
    latest_jpeg_.assign(reinterpret_cast<const char *>(msg->data.data()), msg->data.size());
    latest_image_source_ = get_parameter("compressed_image_topic").as_string();
    has_camera_frame_.store(true);
  }

  void on_image(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    cv::Mat bgr;
    if (!image_to_bgr(*msg, bgr)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000, "Unsupported RGB image encoding: %s",
        msg->encoding.c_str());
      return;
    }

    std::vector<unsigned char> encoded;
    const int quality = std::clamp(get_parameter("jpeg_quality").as_int(), 30L, 95L);
    const std::vector<int> params{cv::IMWRITE_JPEG_QUALITY, quality};
    if (!cv::imencode(".jpg", bgr, encoded, params)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "Failed to encode RGB frame as JPEG.");
      return;
    }

    std::lock_guard<std::mutex> lock(image_mutex_);
    if (latest_image_source_ == get_parameter("compressed_image_topic").as_string()) {
      return;
    }
    latest_jpeg_.assign(reinterpret_cast<const char *>(encoded.data()), encoded.size());
    latest_image_source_ = get_parameter("rgb_image_topic").as_string();
    has_camera_frame_.store(true);
  }

  bool image_to_bgr(const sensor_msgs::msg::Image & msg, cv::Mat & bgr)
  {
    if (msg.width == 0 || msg.height == 0 || msg.data.empty()) {
      return false;
    }

    if (msg.encoding == "bgr8") {
      bgr = cv::Mat(
        static_cast<int>(msg.height), static_cast<int>(msg.width), CV_8UC3,
        const_cast<unsigned char *>(msg.data.data()), msg.step).clone();
      return true;
    }
    if (msg.encoding == "rgb8") {
      const cv::Mat rgb(
        static_cast<int>(msg.height), static_cast<int>(msg.width), CV_8UC3,
        const_cast<unsigned char *>(msg.data.data()), msg.step);
      cv::cvtColor(rgb, bgr, cv::COLOR_RGB2BGR);
      return true;
    }
    if (msg.encoding == "mono8") {
      bgr = cv::Mat(
        static_cast<int>(msg.height), static_cast<int>(msg.width), CV_8UC1,
        const_cast<unsigned char *>(msg.data.data()), msg.step).clone();
      return true;
    }
    if (msg.encoding == "rgba8") {
      const cv::Mat rgba(
        static_cast<int>(msg.height), static_cast<int>(msg.width), CV_8UC4,
        const_cast<unsigned char *>(msg.data.data()), msg.step);
      cv::cvtColor(rgba, bgr, cv::COLOR_RGBA2BGR);
      return true;
    }
    if (msg.encoding == "bgra8") {
      const cv::Mat bgra(
        static_cast<int>(msg.height), static_cast<int>(msg.width), CV_8UC4,
        const_cast<unsigned char *>(msg.data.data()), msg.step);
      cv::cvtColor(bgra, bgr, cv::COLOR_BGRA2BGR);
      return true;
    }

    return false;
  }

  struct GridSnapshot
  {
    bool available{false};
    std::string frame_id;
    uint32_t width{0};
    uint32_t height{0};
    uint32_t source_width{0};
    uint32_t source_height{0};
    uint32_t downsample{1};
    double resolution{0.0};
    double origin_x{0.0};
    double origin_y{0.0};
    double origin_yaw{0.0};
    std::vector<int8_t> data;
  };

  struct CloudSnapshot
  {
    bool available{false};
    std::string frame_id;
    std::string source_frame_id;
    bool projected_to_floor{false};
    std::string floor_plane_source;
    double floor_plane_inlier_ratio{0.0};
    uint32_t source_points{0};
    std::vector<std::array<float, 3>> points;
  };

  struct FloorBasis
  {
    bool valid{false};
    std::array<double, 3> u{1.0, 0.0, 0.0};
    std::array<double, 3> v{0.0, 1.0, 0.0};
    std::array<double, 3> normal{0.0, 0.0, 1.0};
    double d{0.0};
    std::string source{"configured"};
    size_t inliers{0};
    double inlier_ratio{0.0};
  };

  static double dot3(const std::array<double, 3> & a, const std::array<double, 3> & b)
  {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
  }

  static std::array<double, 3> cross3(
    const std::array<double, 3> & a, const std::array<double, 3> & b)
  {
    return {
      a[1] * b[2] - a[2] * b[1],
      a[2] * b[0] - a[0] * b[2],
      a[0] * b[1] - a[1] * b[0]};
  }

  static bool normalize3(std::array<double, 3> & value)
  {
    const double norm = std::sqrt(dot3(value, value));
    if (norm < 1e-12) {
      return false;
    }
    value[0] /= norm;
    value[1] /= norm;
    value[2] /= norm;
    return true;
  }

  FloorBasis floor_basis_from_plane(
    const std::array<double, 4> & plane,
    const std::string & source,
    const size_t inliers = 0,
    const double inlier_ratio = 0.0) const
  {
    FloorBasis basis;
    basis.normal = {plane[0], plane[1], plane[2]};
    basis.d = plane[3];
    const double norm = std::sqrt(dot3(basis.normal, basis.normal));
    if (norm < 1e-12) {
      return basis;
    }
    basis.normal[0] /= norm;
    basis.normal[1] /= norm;
    basis.normal[2] /= norm;
    basis.d /= norm;
    if (!normalize3(basis.normal)) {
      return basis;
    }
    if (basis.normal[2] < 0.0) {
      basis.normal[0] = -basis.normal[0];
      basis.normal[1] = -basis.normal[1];
      basis.normal[2] = -basis.normal[2];
      basis.d = -basis.d;
    }

    std::array<double, 3> seed{1.0, 0.0, 0.0};
    if (std::abs(basis.normal[0]) > 0.9) {
      seed = {0.0, 1.0, 0.0};
    }
    const double seed_dot_normal = dot3(seed, basis.normal);
    basis.u = {
      seed[0] - seed_dot_normal * basis.normal[0],
      seed[1] - seed_dot_normal * basis.normal[1],
      seed[2] - seed_dot_normal * basis.normal[2]};
    if (!normalize3(basis.u)) {
      return basis;
    }
    basis.v = cross3(basis.normal, basis.u);
    if (!normalize3(basis.v)) {
      return basis;
    }
    basis.valid = true;
    basis.source = source;
    basis.inliers = inliers;
    basis.inlier_ratio = inlier_ratio;
    return basis;
  }

  FloorBasis floor_basis_from_params() const
  {
    const auto plane = get_parameter("floor_plane").as_double_array();
    if (plane.size() != 4) {
      return {};
    }
    return floor_basis_from_plane({plane[0], plane[1], plane[2], plane[3]}, "configured");
  }

  FloorBasis floor_basis_from_metadata_param() const
  {
    const auto metadata_path = fs::path(get_parameter("map_metadata_path").as_string());
    if (!metadata_path.empty()) {
      const std::string metadata = read_text_file_quiet(metadata_path);
      if (!metadata.empty()) {
        const auto basis = floor_basis_from_plane(json_floor_plane_value(metadata), "metadata");
        if (basis.valid) {
          return basis;
        }
      }
    }
    return floor_basis_from_params();
  }

  FloorBasis fit_floor_basis_from_cloud(const CloudSnapshot & cloud) const
  {
    FloorBasis empty;
    if (!get_parameter("auto_fit_floor_plane").as_bool() || !cloud.available ||
      cloud.points.size() < 50)
    {
      return empty;
    }

    const size_t source_count = cloud.points.size();
    const size_t max_fit_points = static_cast<size_t>(
      std::max<int64_t>(50, get_parameter("floor_plane_fit_max_points").as_int()));
    const size_t stride = std::max<size_t>(1, (source_count + max_fit_points - 1) / max_fit_points);
    std::vector<std::array<double, 3>> points;
    points.reserve(std::min(source_count, max_fit_points));
    for (size_t i = 0; i < source_count && points.size() < max_fit_points; i += stride) {
      const auto & p = cloud.points[i];
      if (std::isfinite(p[0]) && std::isfinite(p[1]) && std::isfinite(p[2])) {
        points.push_back({static_cast<double>(p[0]), static_cast<double>(p[1]), static_cast<double>(p[2])});
      }
    }
    if (points.size() < 50) {
      return empty;
    }

    const int iterations = std::max<int64_t>(20, get_parameter("floor_plane_fit_iterations").as_int());
    const double threshold = std::max(0.01, get_parameter("floor_plane_fit_distance_threshold").as_double());
    const double min_ratio = std::clamp(
      get_parameter("floor_plane_fit_min_inlier_ratio").as_double(), 0.01, 0.95);
    const double min_abs_normal_z = std::clamp(
      get_parameter("floor_plane_fit_min_abs_normal_z").as_double(), 0.05, 0.99);
    const double height_band = std::max(
      threshold * 1.5, get_parameter("floor_plane_fit_height_band").as_double());

    double min_z = std::numeric_limits<double>::max();
    double max_z = std::numeric_limits<double>::lowest();
    for (const auto & p : points) {
      min_z = std::min(min_z, p[2]);
      max_z = std::max(max_z, p[2]);
    }

    std::vector<std::array<double, 3>> sample_points;
    if (std::isfinite(min_z) && std::isfinite(max_z) && max_z > min_z) {
      const double bin_size = std::max(0.04, threshold);
      const size_t bin_count = std::min<size_t>(
        240, std::max<size_t>(1, static_cast<size_t>(std::ceil((max_z - min_z) / bin_size)) + 1));
      std::vector<size_t> histogram(bin_count, 0);
      for (const auto & p : points) {
        const auto bin = std::min<size_t>(
          bin_count - 1,
          static_cast<size_t>(std::max(0.0, std::floor((p[2] - min_z) / bin_size))));
        ++histogram[bin];
      }
      const auto best_bin_it = std::max_element(histogram.begin(), histogram.end());
      const size_t max_bin_count = best_bin_it == histogram.end() ? 0 : *best_bin_it;
      const double min_height_bin_ratio = std::clamp(
        get_parameter("floor_plane_fit_min_height_bin_ratio").as_double(), 0.001, 0.2);
      const double min_peak_ratio = std::clamp(
        get_parameter("floor_plane_fit_min_peak_ratio").as_double(), 0.05, 0.95);
      const size_t min_candidate_count = std::max<size_t>(
        30,
        std::max<size_t>(
          static_cast<size_t>(std::ceil(static_cast<double>(points.size()) * min_height_bin_ratio)),
          static_cast<size_t>(std::ceil(static_cast<double>(max_bin_count) * min_peak_ratio))));

      size_t floor_bin = 0;
      bool found_floor_bin = false;
      for (size_t bin = 0; bin < histogram.size(); ++bin) {
        if (histogram[bin] >= min_candidate_count) {
          floor_bin = bin;
          found_floor_bin = true;
          break;
        }
      }
      if (!found_floor_bin && best_bin_it != histogram.end()) {
        floor_bin = static_cast<size_t>(std::distance(histogram.begin(), best_bin_it));
      }

      const double dominant_z = min_z + (static_cast<double>(floor_bin) + 0.5) * bin_size;
      for (const auto & p : points) {
        if (std::abs(p[2] - dominant_z) <= height_band) {
          sample_points.push_back(p);
        }
      }
    }
    if (sample_points.size() < 50) {
      sample_points = points;
    }

    size_t best_inliers = 0;
    std::array<double, 3> best_normal{0.0, 0.0, 1.0};
    double best_d = 0.0;
    const size_t n = sample_points.size();
    std::mt19937 rng(20260626);
    std::uniform_int_distribution<size_t> pick(0, n - 1);
    for (int iter = 0; iter < iterations; ++iter) {
      const size_t i1 = pick(rng);
      const size_t i2 = pick(rng);
      const size_t i3 = pick(rng);
      if (i1 == i2 || i1 == i3 || i2 == i3) {
        continue;
      }
      const auto & p1 = sample_points[i1];
      const auto & p2 = sample_points[i2];
      const auto & p3 = sample_points[i3];
      const std::array<double, 3> a{p2[0] - p1[0], p2[1] - p1[1], p2[2] - p1[2]};
      const std::array<double, 3> b{p3[0] - p1[0], p3[1] - p1[1], p3[2] - p1[2]};
      auto normal = cross3(a, b);
      if (!normalize3(normal) || std::abs(normal[2]) < min_abs_normal_z) {
        continue;
      }
      double d = -dot3(normal, p1);
      if (normal[2] < 0.0) {
        normal[0] = -normal[0];
        normal[1] = -normal[1];
        normal[2] = -normal[2];
        d = -d;
      }

      size_t inliers = 0;
      for (const auto & p : points) {
        const double distance = std::abs(dot3(normal, p) + d);
        if (distance <= threshold) {
          ++inliers;
        }
      }
      if (inliers > best_inliers) {
        best_inliers = inliers;
        best_normal = normal;
        best_d = d;
      }
    }

    const double ratio = static_cast<double>(best_inliers) / static_cast<double>(points.size());
    if (best_inliers < 50 || ratio < min_ratio) {
      return empty;
    }
    return floor_basis_from_plane(
      {best_normal[0], best_normal[1], best_normal[2], best_d}, "auto_fit", best_inliers, ratio);
  }

  FloorBasis floor_basis_for_cloud(const CloudSnapshot & cloud) const
  {
    auto fitted = fit_floor_basis_from_cloud(cloud);
    if (fitted.valid) {
      return fitted;
    }
    return floor_basis_from_params();
  }

  void project_cloud_to_floor_frame(CloudSnapshot & cloud, const FloorBasis & basis) const
  {
    if (!cloud.available || cloud.points.empty()) {
      return;
    }
    if (!basis.valid) {
      return;
    }

    if (cloud.source_frame_id.empty()) {
      cloud.source_frame_id = cloud.frame_id;
    }
    for (auto & point : cloud.points) {
      const std::array<double, 3> p{
        static_cast<double>(point[0]),
        static_cast<double>(point[1]),
        static_cast<double>(point[2])};
      point[0] = static_cast<float>(dot3(p, basis.u));
      point[1] = static_cast<float>(dot3(p, basis.v));
      point[2] = static_cast<float>(dot3(p, basis.normal) + basis.d);
    }
    cloud.frame_id = "floor";
    cloud.projected_to_floor = true;
    cloud.floor_plane_source = basis.source;
    cloud.floor_plane_inlier_ratio = basis.inlier_ratio;
  }

  void filter_floor_height(CloudSnapshot & cloud, const double min_z, const double max_z) const
  {
    if (!cloud.available || !cloud.projected_to_floor) {
      return;
    }
    std::vector<std::array<float, 3>> filtered;
    filtered.reserve(cloud.points.size());
    for (const auto & point : cloud.points) {
      const double z = static_cast<double>(point[2]);
      if (std::isfinite(z) && z >= min_z && z <= max_z) {
        filtered.push_back(point);
      }
    }
    cloud.points = std::move(filtered);
  }

  bool find_float32_field_offset(
    const sensor_msgs::msg::PointCloud2 & msg, const std::string & name, uint32_t & offset) const
  {
    for (const auto & field : msg.fields) {
      if (field.name == name && field.datatype == sensor_msgs::msg::PointField::FLOAT32 &&
        field.count == 1)
      {
        offset = field.offset;
        return true;
      }
    }
    return false;
  }

  bool read_float32_field(
    const sensor_msgs::msg::PointCloud2 & msg, const size_t point_index,
    const uint32_t offset, float & value) const
  {
    const size_t byte_index = point_index * static_cast<size_t>(msg.point_step) + offset;
    if (byte_index + sizeof(float) > msg.data.size()) {
      return false;
    }
    std::memcpy(&value, msg.data.data() + byte_index, sizeof(float));
    return true;
  }

  bool sample_point_cloud(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg,
    const uint32_t max_points,
    CloudSnapshot & next)
  {
    const uint64_t source_points =
      static_cast<uint64_t>(msg->width) * static_cast<uint64_t>(msg->height);
    if (source_points == 0 || msg->point_step == 0 || msg->data.empty()) {
      return false;
    }

    uint32_t x_offset = 0;
    uint32_t y_offset = 0;
    uint32_t z_offset = 0;
    if (!find_float32_field_offset(*msg, "x", x_offset) ||
      !find_float32_field_offset(*msg, "y", y_offset) ||
      !find_float32_field_offset(*msg, "z", z_offset))
    {
      return false;
    }

    next.available = true;
    next.frame_id = msg->header.frame_id;
    next.source_frame_id = msg->header.frame_id;
    next.source_points = static_cast<uint32_t>(std::min<uint64_t>(
        source_points, std::numeric_limits<uint32_t>::max()));
    next.points.clear();
    next.points.reserve(static_cast<size_t>(std::min<uint64_t>(source_points, max_points)));

    const uint64_t stride = std::max<uint64_t>(
      1, (source_points + static_cast<uint64_t>(max_points) - 1) /
      static_cast<uint64_t>(max_points));
    for (uint64_t source_index = 0;
      source_index < source_points && next.points.size() < max_points;
      source_index += stride)
    {
      float x = 0.0f;
      float y = 0.0f;
      float z = 0.0f;
      const size_t point_index = static_cast<size_t>(source_index);
      if (!read_float32_field(*msg, point_index, x_offset, x) ||
        !read_float32_field(*msg, point_index, y_offset, y) ||
        !read_float32_field(*msg, point_index, z_offset, z))
      {
        continue;
      }
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
        continue;
      }
      next.points.push_back({x, y, z});
    }

    return true;
  }

  struct PoseSnapshot
  {
    bool available{false};
    std::string frame_id;
    std::string source;
    double x{0.0};
    double y{0.0};
    double z{0.0};
    double yaw{0.0};
  };

  struct PathSnapshot
  {
    bool available{false};
    std::string frame_id;
    std::string source;
    std::vector<std::array<float, 3>> points;
  };

  void on_map(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    const uint32_t source_width = msg->info.width;
    const uint32_t source_height = msg->info.height;
    const uint64_t source_cells =
      static_cast<uint64_t>(source_width) * static_cast<uint64_t>(source_height);
    if (source_width == 0 || source_height == 0 || source_cells == 0 || msg->data.empty()) {
      return;
    }

    const int64_t max_cells = std::max<int64_t>(1, get_parameter("max_grid_cells").as_int());
    uint32_t downsample = 1;
    while (
      (static_cast<uint64_t>((source_width + downsample - 1) / downsample) *
      static_cast<uint64_t>((source_height + downsample - 1) / downsample)) >
      static_cast<uint64_t>(max_cells))
    {
      ++downsample;
    }

    GridSnapshot next;
    next.available = true;
    next.frame_id = msg->header.frame_id;
    next.source_width = source_width;
    next.source_height = source_height;
    next.downsample = downsample;
    next.width = (source_width + downsample - 1) / downsample;
    next.height = (source_height + downsample - 1) / downsample;
    next.resolution = msg->info.resolution * static_cast<double>(downsample);
    next.origin_x = msg->info.origin.position.x;
    next.origin_y = msg->info.origin.position.y;
    next.origin_yaw = yaw_from_quaternion(
      msg->info.origin.orientation.x,
      msg->info.origin.orientation.y,
      msg->info.origin.orientation.z,
      msg->info.origin.orientation.w);
    next.data.reserve(static_cast<size_t>(next.width) * static_cast<size_t>(next.height));

    for (uint32_t y = 0; y < next.height; ++y) {
      const uint32_t source_y = std::min(y * downsample, source_height - 1);
      for (uint32_t x = 0; x < next.width; ++x) {
        const uint32_t source_x = std::min(x * downsample, source_width - 1);
        const auto source_index =
          static_cast<size_t>(source_y) * static_cast<size_t>(source_width) +
          static_cast<size_t>(source_x);
        next.data.push_back(msg->data[source_index]);
      }
    }

    std::lock_guard<std::mutex> lock(map_mutex_);
    grid_ = std::move(next);
  }

  void on_nav_costmap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    const uint32_t source_width = msg->info.width;
    const uint32_t source_height = msg->info.height;
    const uint64_t source_cells =
      static_cast<uint64_t>(source_width) * static_cast<uint64_t>(source_height);
    if (source_width == 0 || source_height == 0 || source_cells == 0 || msg->data.empty()) {
      return;
    }

    const int64_t max_cells = std::max<int64_t>(1, get_parameter("max_grid_cells").as_int());
    uint32_t downsample = 1;
    while (
      (static_cast<uint64_t>((source_width + downsample - 1) / downsample) *
      static_cast<uint64_t>((source_height + downsample - 1) / downsample)) >
      static_cast<uint64_t>(max_cells))
    {
      ++downsample;
    }

    GridSnapshot next;
    next.available = true;
    next.frame_id = msg->header.frame_id;
    next.source_width = source_width;
    next.source_height = source_height;
    next.downsample = downsample;
    next.width = (source_width + downsample - 1) / downsample;
    next.height = (source_height + downsample - 1) / downsample;
    next.resolution = msg->info.resolution * static_cast<double>(downsample);
    next.origin_x = msg->info.origin.position.x;
    next.origin_y = msg->info.origin.position.y;
    next.origin_yaw = yaw_from_quaternion(
      msg->info.origin.orientation.x,
      msg->info.origin.orientation.y,
      msg->info.origin.orientation.z,
      msg->info.origin.orientation.w);
    next.data.reserve(static_cast<size_t>(next.width) * static_cast<size_t>(next.height));

    for (uint32_t y = 0; y < next.height; ++y) {
      const uint32_t source_y = std::min(y * downsample, source_height - 1);
      for (uint32_t x = 0; x < next.width; ++x) {
        const uint32_t source_x = std::min(x * downsample, source_width - 1);
        const auto source_index =
          static_cast<size_t>(source_y) * static_cast<size_t>(source_width) +
          static_cast<size_t>(source_x);
        next.data.push_back(msg->data[source_index]);
      }
    }

    std::lock_guard<std::mutex> lock(map_mutex_);
    nav_costmap_ = std::move(next);
  }

  void on_cloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg, const bool is_static)
  {
    const uint32_t source_points = msg->width * msg->height;
    if (source_points == 0) {
      return;
    }

    const int64_t configured_max = is_static ?
      get_parameter("max_static_cloud_points").as_int() :
      get_parameter("max_live_cloud_points").as_int();
    const uint32_t max_points = static_cast<uint32_t>(std::max<int64_t>(1, configured_max));
    CloudSnapshot next;
    if (!sample_point_cloud(msg, max_points, next)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000, "PointCloud2 parse failed or missing float32 x/y/z fields");
      return;
    }

    std::lock_guard<std::mutex> lock(map_mutex_);
    if (is_static) {
      static_cloud_ = std::move(next);
    } else {
      live_cloud_ = std::move(next);
    }
  }

  void on_cloud_3d(const sensor_msgs::msg::PointCloud2::SharedPtr msg, const bool is_static)
  {
    const uint32_t source_points = msg->width * msg->height;
    if (source_points == 0) {
      return;
    }

    const int64_t configured_max = is_static ?
      get_parameter("max_static_cloud_points").as_int() :
      get_parameter("max_live_cloud_points").as_int();
    const uint32_t max_points = static_cast<uint32_t>(std::max<int64_t>(1, configured_max));
    CloudSnapshot next;
    if (!sample_point_cloud(msg, max_points, next)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "3D PointCloud2 parse failed or missing float32 x/y/z fields");
      return;
    }
    // Localized 3D clouds are already in the map frame. Use the current map
    // metadata plane so the browser reference plane matches the loaded map.
    project_cloud_to_floor_frame(next, floor_basis_from_metadata_param());

    std::lock_guard<std::mutex> lock(map_mutex_);
    if (is_static) {
      static_cloud_3d_ = std::move(next);
    } else {
      live_cloud_3d_ = std::move(next);
    }
  }

  void on_mapping_cloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    rclcpp::Time mapping_started_at;
    bool mapping_start_time_valid = false;
    {
      std::lock_guard<std::mutex> lock(mapping_mutex_);
      if (!mapping_active_) {
        return;
      }
      mapping_started_at = mapping_started_at_;
      mapping_start_time_valid = mapping_start_time_valid_;
    }

    if (mapping_start_time_valid && msg->header.stamp.sec != 0) {
      const rclcpp::Time msg_time(msg->header.stamp);
      if (msg_time < mapping_started_at) {
        return;
      }
    }

    const uint32_t source_points = msg->width * msg->height;
    if (source_points == 0) {
      return;
    }

    const int64_t configured_max = get_parameter("max_mapping_cloud_points").as_int();
    const uint32_t max_points = configured_max <= 0 ?
      source_points :
      static_cast<uint32_t>(std::max<int64_t>(1, configured_max));

    CloudSnapshot next;
    CloudSnapshot next_3d;
    if (!sample_point_cloud(msg, max_points, next)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "Mapping PointCloud2 parse failed or missing float32 x/y/z fields");
      return;
    }
    next_3d = next;
    const auto mapping_basis = floor_basis_for_cloud(next);
    if (get_parameter("project_mapping_cloud_to_floor").as_bool()) {
      project_cloud_to_floor_frame(next, mapping_basis);
    }
    project_cloud_to_floor_frame(next_3d, mapping_basis);

    std::lock_guard<std::mutex> lock(map_mutex_);
    mapping_cloud_ = std::move(next);
    mapping_cloud_3d_ = std::move(next_3d);
    mapping_floor_basis_ = mapping_basis;
  }

  void clear_mapping_clouds()
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    mapping_cloud_ = CloudSnapshot{};
    mapping_cloud_3d_ = CloudSnapshot{};
    mapping_floor_basis_ = FloorBasis{};
  }

  std::string restart_fast_lio_for_mapping()
  {
    if (!get_parameter("restart_fast_lio_on_mapping_start").as_bool()) {
      return "FAST-LIO重启已按参数关闭。";
    }

    const auto process_match = get_parameter("fast_lio_process_match").as_string();
    if (process_match.empty()) {
      return "FAST-LIO进程匹配参数为空，未请求重启。";
    }

    DIR * proc_dir = opendir("/proc");
    if (!proc_dir) {
      return "无法读取 /proc，未请求FAST-LIO重启。";
    }

    const pid_t self_pid = getpid();
    int matched = 0;
    int signaled = 0;
    while (dirent * entry = readdir(proc_dir)) {
      char * end = nullptr;
      errno = 0;
      const long pid_long = std::strtol(entry->d_name, &end, 10);
      if (errno != 0 || end == entry->d_name || *end != '\0' || pid_long <= 1) {
        continue;
      }
      const auto pid = static_cast<pid_t>(pid_long);
      if (pid == self_pid) {
        continue;
      }

      const fs::path cmdline_path = fs::path("/proc") / entry->d_name / "cmdline";
      std::ifstream cmdline_file(cmdline_path, std::ios::binary);
      if (!cmdline_file) {
        continue;
      }
      std::ostringstream buffer;
      buffer << cmdline_file.rdbuf();
      std::string cmdline = buffer.str();
      if (cmdline.empty()) {
        continue;
      }
      std::replace(cmdline.begin(), cmdline.end(), '\0', ' ');
      if (cmdline.find(process_match) == std::string::npos) {
        continue;
      }
      if (cmdline.find("web_api_node") != std::string::npos) {
        continue;
      }

      matched += 1;
      if (kill(pid, SIGINT) == 0) {
        signaled += 1;
      }
    }
    closedir(proc_dir);

    if (matched == 0) {
      RCLCPP_WARN(
        get_logger(), "No FAST-LIO process matched '%s' for mapping restart",
        process_match.c_str());
      return "未找到FAST-LIO进程，网页已从当前时刻重新接收建图点云。";
    }
    if (signaled == matched) {
      RCLCPP_INFO(
        get_logger(), "Requested FAST-LIO restart by SIGINT for %d process(es)", signaled);
      return "已请求FAST-LIO重启，launch会自动拉起新建图进程。";
    }

    RCLCPP_WARN(
      get_logger(), "Requested FAST-LIO restart matched %d process(es), signaled %d",
      matched, signaled);
    std::ostringstream message;
    message << "FAST-LIO重启请求部分成功：" << signaled << "/" << matched << " 个进程已通知。";
    return message.str();
  }

  void project_pose_to_floor_frame(PoseSnapshot & pose, const FloorBasis & basis) const
  {
    if (!pose.available) {
      return;
    }
    if (!basis.valid) {
      return;
    }

    const std::array<double, 3> position{pose.x, pose.y, pose.z};
    const std::array<double, 3> heading{std::cos(pose.yaw), std::sin(pose.yaw), 0.0};
    pose.x = dot3(position, basis.u);
    pose.y = dot3(position, basis.v);
    pose.z = dot3(position, basis.normal) + basis.d;
    pose.yaw = std::atan2(dot3(heading, basis.v), dot3(heading, basis.u));
    pose.frame_id = "floor";
  }

  void on_pose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    PoseSnapshot next;
    next.available = true;
    next.frame_id = msg->header.frame_id;
    next.source = get_parameter("pose_topic").as_string();
    next.x = msg->pose.position.x;
    next.y = msg->pose.position.y;
    next.z = msg->pose.position.z;
    next.yaw = yaw_from_quaternion(
      msg->pose.orientation.x,
      msg->pose.orientation.y,
      msg->pose.orientation.z,
      msg->pose.orientation.w);

    std::lock_guard<std::mutex> lock(map_mutex_);
    pose_ = next;
  }

  void on_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    PoseSnapshot next;
    next.available = true;
    next.frame_id = msg->header.frame_id;
    next.source = get_parameter("odom_topic").as_string();
    next.x = msg->pose.pose.position.x;
    next.y = msg->pose.pose.position.y;
    next.z = msg->pose.pose.position.z;
    next.yaw = yaw_from_quaternion(
      msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z,
      msg->pose.pose.orientation.w);

    std::lock_guard<std::mutex> lock(map_mutex_);
    odom_pose_ = next;
  }

  void on_nav_path(const nav_msgs::msg::Path::SharedPtr msg)
  {
    PathSnapshot next;
    next.available = !msg->poses.empty();
    next.frame_id = msg->header.frame_id;
    next.source = get_parameter("nav_path_topic").as_string();
    next.points.reserve(msg->poses.size());
    for (const auto & pose : msg->poses) {
      next.points.push_back({
        static_cast<float>(pose.pose.position.x),
        static_cast<float>(pose.pose.position.y),
        static_cast<float>(pose.pose.position.z)});
    }

    std::lock_guard<std::mutex> lock(map_mutex_);
    nav_path_ = std::move(next);
  }

  void on_nav_local_path(const nav_msgs::msg::Path::SharedPtr msg)
  {
    PathSnapshot next;
    next.available = !msg->poses.empty();
    next.frame_id = msg->header.frame_id;
    next.source = get_parameter("nav_local_path_topic").as_string();
    next.points.reserve(msg->poses.size());
    for (const auto & pose : msg->poses) {
      next.points.push_back({
        static_cast<float>(pose.pose.position.x),
        static_cast<float>(pose.pose.position.y),
        static_cast<float>(pose.pose.position.z)});
    }

    std::lock_guard<std::mutex> lock(map_mutex_);
    nav_local_path_ = std::move(next);
  }

  void on_nav_cmd(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    if (!nav_drive_active_.load()) {
      return;
    }
    cmd_pub_->publish(*msg);
  }

  std::string make_map_status_json()
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    std::ostringstream json;
    json << "{";
    json << "\"ok\":true,";
    json << "\"has_grid\":" << (grid_.available ? "true" : "false") << ",";
    json << "\"has_static_cloud\":" << (static_cloud_.available ? "true" : "false") << ",";
    json << "\"has_live_cloud\":" << (live_cloud_.available ? "true" : "false") << ",";
    json << "\"has_mapping_cloud\":" << (mapping_cloud_.available ? "true" : "false") << ",";
    json << "\"has_pose\":" << (pose_.available ? "true" : "false") << ",";
    json << "\"has_odom_pose\":" << (odom_pose_.available ? "true" : "false") << ",";
    json << "\"has_nav_path\":" << (nav_path_.available ? "true" : "false") << ",";
    json << "\"has_nav_local_path\":" << (nav_local_path_.available ? "true" : "false") << ",";
    json << "\"has_nav_costmap\":" << (nav_costmap_.available ? "true" : "false") << ",";
    json << "\"nav_drive_active\":" << (nav_drive_active_.load() ? "true" : "false") << ",";
    json << "\"grid_topic\":\"" << json_escape(get_parameter("map_topic").as_string()) << "\",";
    json << "\"static_cloud_topic\":\"" <<
      json_escape(get_parameter("static_cloud_topic").as_string()) << "\",";
    json << "\"live_cloud_topic\":\"" <<
      json_escape(get_parameter("live_cloud_topic").as_string()) << "\",";
    json << "\"mapping_cloud_topic\":\"" <<
      json_escape(get_parameter("mapping_cloud_topic").as_string()) << "\",";
    json << "\"pose_topic\":\"" << json_escape(get_parameter("pose_topic").as_string()) << "\",";
    json << "\"odom_topic\":\"" << json_escape(get_parameter("odom_topic").as_string()) << "\",";
    json << "\"nav_path_topic\":\"" << json_escape(get_parameter("nav_path_topic").as_string()) << "\",";
    json << "\"nav_local_path_topic\":\"" << json_escape(get_parameter("nav_local_path_topic").as_string()) << "\",";
    json << "\"nav_costmap_topic\":\"" << json_escape(get_parameter("nav_costmap_topic").as_string()) << "\",";
    json << "\"localization_status\":\"" << json_escape(localization_status_) << "\",";
    json << "\"nav_status\":\"" << json_escape(nav_status_) << "\",";
    json << "\"grid_cells\":" << grid_.data.size() << ",";
    json << "\"static_cloud_points\":" << static_cloud_.points.size() << ",";
    json << "\"live_cloud_points\":" << live_cloud_.points.size() << ",";
    json << "\"mapping_cloud_points\":" << mapping_cloud_.points.size() << ",";
    json << "\"nav_path_points\":" << nav_path_.points.size() << ",";
    json << "\"nav_local_path_points\":" << nav_local_path_.points.size() << ",";
    json << "\"nav_costmap_cells\":" << nav_costmap_.data.size();
    json << "}";
    return json.str();
  }

  void append_points_json(std::ostringstream & json, const std::vector<std::array<float, 3>> & points)
  {
    json << "[";
    for (size_t i = 0; i < points.size(); ++i) {
      if (i > 0) {
        json << ",";
      }
      json << "[" << points[i][0] << "," << points[i][1] << "," << points[i][2] << "]";
    }
    json << "]";
  }

  std::string make_map_snapshot_json()
  {
    GridSnapshot grid;
    CloudSnapshot static_cloud;
    CloudSnapshot live_cloud;
    CloudSnapshot mapping_cloud;
    CloudSnapshot static_cloud_3d;
    CloudSnapshot live_cloud_3d;
    CloudSnapshot mapping_cloud_3d;
    PoseSnapshot pose;
    PoseSnapshot odom_pose;
    FloorBasis mapping_floor_basis;
    PathSnapshot nav_path;
    PathSnapshot nav_local_path;
    GridSnapshot nav_costmap;
    std::string localization_status;
    std::string nav_status;
    {
      std::lock_guard<std::mutex> lock(map_mutex_);
      grid = grid_;
      static_cloud = static_cloud_;
      live_cloud = live_cloud_;
      mapping_cloud = mapping_cloud_;
      static_cloud_3d = static_cloud_3d_;
      live_cloud_3d = live_cloud_3d_;
      mapping_cloud_3d = mapping_cloud_3d_;
      mapping_floor_basis = mapping_floor_basis_;
      pose = pose_;
      odom_pose = odom_pose_;
      nav_path = nav_path_;
      nav_local_path = nav_local_path_;
      nav_costmap = nav_costmap_;
      localization_status = localization_status_;
      nav_status = nav_status_;
    }
    if (mapping_cloud.projected_to_floor && mapping_floor_basis.valid) {
      project_pose_to_floor_frame(odom_pose, mapping_floor_basis);
    }

    std::ostringstream json;
    json << std::fixed << std::setprecision(3);
    json << "{";
    json << "\"ok\":true,";

    json << "\"grid\":{";
    json << "\"available\":" << (grid.available ? "true" : "false") << ",";
    json << "\"frame_id\":\"" << json_escape(grid.frame_id) << "\",";
    json << "\"width\":" << grid.width << ",";
    json << "\"height\":" << grid.height << ",";
    json << "\"source_width\":" << grid.source_width << ",";
    json << "\"source_height\":" << grid.source_height << ",";
    json << "\"downsample\":" << grid.downsample << ",";
    json << "\"resolution\":" << grid.resolution << ",";
    json << "\"origin\":{\"x\":" << grid.origin_x << ",\"y\":" << grid.origin_y <<
      ",\"yaw\":" << grid.origin_yaw << "},";
    json << "\"data\":[";
    for (size_t i = 0; i < grid.data.size(); ++i) {
      if (i > 0) {
        json << ",";
      }
      json << static_cast<int>(grid.data[i]);
    }
    json << "]},";  // grid

    json << "\"nav_costmap\":{";
    json << "\"available\":" << (nav_costmap.available ? "true" : "false") << ",";
    json << "\"frame_id\":\"" << json_escape(nav_costmap.frame_id) << "\",";
    json << "\"width\":" << nav_costmap.width << ",";
    json << "\"height\":" << nav_costmap.height << ",";
    json << "\"source_width\":" << nav_costmap.source_width << ",";
    json << "\"source_height\":" << nav_costmap.source_height << ",";
    json << "\"downsample\":" << nav_costmap.downsample << ",";
    json << "\"resolution\":" << nav_costmap.resolution << ",";
    json << "\"origin\":{\"x\":" << nav_costmap.origin_x << ",\"y\":" << nav_costmap.origin_y <<
      ",\"yaw\":" << nav_costmap.origin_yaw << "},";
    json << "\"data\":[";
    for (size_t i = 0; i < nav_costmap.data.size(); ++i) {
      if (i > 0) {
        json << ",";
      }
      json << static_cast<int>(nav_costmap.data[i]);
    }
    json << "]},";  // nav_costmap

    json << "\"static_cloud\":{";
    json << "\"available\":" << (static_cloud.available ? "true" : "false") << ",";
    json << "\"frame_id\":\"" << json_escape(static_cloud.frame_id) << "\",";
    json << "\"source_frame_id\":\"" << json_escape(static_cloud.source_frame_id) << "\",";
    json << "\"projected_to_floor\":" << (static_cloud.projected_to_floor ? "true" : "false") << ",";
    json << "\"floor_plane_source\":\"" << json_escape(static_cloud.floor_plane_source) << "\",";
    json << "\"floor_plane_inlier_ratio\":" << static_cloud.floor_plane_inlier_ratio << ",";
    json << "\"source_points\":" << static_cloud.source_points << ",";
    json << "\"points\":";
    append_points_json(json, static_cloud.points);
    json << "},";

    json << "\"live_cloud\":{";
    json << "\"available\":" << (live_cloud.available ? "true" : "false") << ",";
    json << "\"frame_id\":\"" << json_escape(live_cloud.frame_id) << "\",";
    json << "\"source_frame_id\":\"" << json_escape(live_cloud.source_frame_id) << "\",";
    json << "\"projected_to_floor\":" << (live_cloud.projected_to_floor ? "true" : "false") << ",";
    json << "\"floor_plane_source\":\"" << json_escape(live_cloud.floor_plane_source) << "\",";
    json << "\"floor_plane_inlier_ratio\":" << live_cloud.floor_plane_inlier_ratio << ",";
    json << "\"source_points\":" << live_cloud.source_points << ",";
    json << "\"points\":";
    append_points_json(json, live_cloud.points);
    json << "},";

    json << "\"mapping_cloud\":{";
    json << "\"available\":" << (mapping_cloud.available ? "true" : "false") << ",";
    json << "\"frame_id\":\"" << json_escape(mapping_cloud.frame_id) << "\",";
    json << "\"source_frame_id\":\"" << json_escape(mapping_cloud.source_frame_id) << "\",";
    json << "\"projected_to_floor\":" << (mapping_cloud.projected_to_floor ? "true" : "false") << ",";
    json << "\"floor_plane_source\":\"" << json_escape(mapping_cloud.floor_plane_source) << "\",";
    json << "\"floor_plane_inlier_ratio\":" << mapping_cloud.floor_plane_inlier_ratio << ",";
    json << "\"source_points\":" << mapping_cloud.source_points << ",";
    json << "\"source\":\"" << json_escape(get_parameter("mapping_cloud_topic").as_string()) << "\",";
    json << "\"points\":";
    append_points_json(json, mapping_cloud.points);
    json << "},";

    json << "\"static_cloud_3d\":{";
    json << "\"available\":" << (static_cloud_3d.available ? "true" : "false") << ",";
    json << "\"frame_id\":\"" << json_escape(static_cloud_3d.frame_id) << "\",";
    json << "\"source_frame_id\":\"" << json_escape(static_cloud_3d.source_frame_id) << "\",";
    json << "\"projected_to_floor\":" << (static_cloud_3d.projected_to_floor ? "true" : "false") << ",";
    json << "\"floor_plane_source\":\"" << json_escape(static_cloud_3d.floor_plane_source) << "\",";
    json << "\"floor_plane_inlier_ratio\":" << static_cloud_3d.floor_plane_inlier_ratio << ",";
    json << "\"source_points\":" << static_cloud_3d.source_points << ",";
    json << "\"points\":";
    append_points_json(json, static_cloud_3d.points);
    json << "},";

    json << "\"live_cloud_3d\":{";
    json << "\"available\":" << (live_cloud_3d.available ? "true" : "false") << ",";
    json << "\"frame_id\":\"" << json_escape(live_cloud_3d.frame_id) << "\",";
    json << "\"source_frame_id\":\"" << json_escape(live_cloud_3d.source_frame_id) << "\",";
    json << "\"projected_to_floor\":" << (live_cloud_3d.projected_to_floor ? "true" : "false") << ",";
    json << "\"floor_plane_source\":\"" << json_escape(live_cloud_3d.floor_plane_source) << "\",";
    json << "\"floor_plane_inlier_ratio\":" << live_cloud_3d.floor_plane_inlier_ratio << ",";
    json << "\"source_points\":" << live_cloud_3d.source_points << ",";
    json << "\"points\":";
    append_points_json(json, live_cloud_3d.points);
    json << "},";

    json << "\"mapping_cloud_3d\":{";
    json << "\"available\":" << (mapping_cloud_3d.available ? "true" : "false") << ",";
    json << "\"frame_id\":\"" << json_escape(mapping_cloud_3d.frame_id) << "\",";
    json << "\"source_frame_id\":\"" << json_escape(mapping_cloud_3d.source_frame_id) << "\",";
    json << "\"projected_to_floor\":" << (mapping_cloud_3d.projected_to_floor ? "true" : "false") << ",";
    json << "\"floor_plane_source\":\"" << json_escape(mapping_cloud_3d.floor_plane_source) << "\",";
    json << "\"floor_plane_inlier_ratio\":" << mapping_cloud_3d.floor_plane_inlier_ratio << ",";
    json << "\"source_points\":" << mapping_cloud_3d.source_points << ",";
    json << "\"source\":\"" << json_escape(get_parameter("mapping_cloud_topic").as_string()) << "\",";
    json << "\"points\":";
    append_points_json(json, mapping_cloud_3d.points);
    json << "},";

    json << "\"pose\":{";
    json << "\"available\":" << (pose.available ? "true" : "false") << ",";
    json << "\"frame_id\":\"" << json_escape(pose.frame_id) << "\",";
    json << "\"source\":\"" << json_escape(pose.source) << "\",";
    json << "\"x\":" << pose.x << ",";
    json << "\"y\":" << pose.y << ",";
    json << "\"z\":" << pose.z << ",";
    json << "\"yaw\":" << pose.yaw;
    json << "},";

    json << "\"odom_pose\":{";
    json << "\"available\":" << (odom_pose.available ? "true" : "false") << ",";
    json << "\"frame_id\":\"" << json_escape(odom_pose.frame_id) << "\",";
    json << "\"source\":\"" << json_escape(odom_pose.source) << "\",";
    json << "\"x\":" << odom_pose.x << ",";
    json << "\"y\":" << odom_pose.y << ",";
    json << "\"z\":" << odom_pose.z << ",";
    json << "\"yaw\":" << odom_pose.yaw;
    json << "},";
    json << "\"nav_path\":{";
    json << "\"available\":" << (nav_path.available ? "true" : "false") << ",";
    json << "\"frame_id\":\"" << json_escape(nav_path.frame_id) << "\",";
    json << "\"source\":\"" << json_escape(nav_path.source) << "\",";
    json << "\"points\":";
    append_points_json(json, nav_path.points);
    json << "},";
    json << "\"nav_local_path\":{";
    json << "\"available\":" << (nav_local_path.available ? "true" : "false") << ",";
    json << "\"frame_id\":\"" << json_escape(nav_local_path.frame_id) << "\",";
    json << "\"source\":\"" << json_escape(nav_local_path.source) << "\",";
    json << "\"points\":";
    append_points_json(json, nav_local_path.points);
    json << "},";
    json << "\"localization_status\":\"" << json_escape(localization_status) << "\",";
    json << "\"nav_status\":\"" << json_escape(nav_status) << "\",";
    json << "\"nav_drive_active\":" << (nav_drive_active_.load() ? "true" : "false");

    json << "}";
    return json.str();
  }

  fs::path saved_maps_dir() const
  {
    return fs::path(get_parameter("saved_maps_dir").as_string());
  }

  fs::path pcd_to_pgm_output_dir() const
  {
    return fs::path(get_parameter("pcd_to_pgm_output_dir").as_string());
  }

  fs::path saved_map_pcd_path(const std::string & map_name) const
  {
    const std::string stem = sanitize_map_stem(map_name);
    if (stem.empty()) {
      return {};
    }
    return saved_maps_dir() / (stem + ".pcd");
  }

  fs::path converted_map_dir(const std::string & map_name) const
  {
    const std::string stem = sanitize_map_stem(map_name);
    if (stem.empty()) {
      return {};
    }
    return pcd_to_pgm_output_dir() / (stem + get_parameter("converted_map_suffix").as_string());
  }

  static bool text_contains_json_bool(
    const std::string & text, const std::string & key, const bool expected)
  {
    const std::string pattern = "\"" + key + "\"";
    const auto key_pos = text.find(pattern);
    if (key_pos == std::string::npos) {
      return false;
    }
    const auto colon = text.find(':', key_pos + pattern.size());
    if (colon == std::string::npos) {
      return false;
    }
    const auto start = text.find_first_not_of(" \t\r\n", colon + 1);
    if (start == std::string::npos) {
      return false;
    }
    return text.compare(start, expected ? 4 : 5, expected ? "true" : "false") == 0;
  }

  static bool text_contains_json_string(
    const std::string & text, const std::string & key, const std::string & expected)
  {
    return text.find("\"" + key + "\"") != std::string::npos &&
      text.find("\"" + expected + "\"") != std::string::npos;
  }

  static double metadata_number_value(
    const std::string & text, const std::string & key, const double fallback)
  {
    try {
      const std::regex pattern("\"" + key + "\"\\s*:\\s*(-?[0-9]+(\\.[0-9]+)?)");
      std::smatch match;
      if (std::regex_search(text, match, pattern)) {
        return std::stod(match[1].str());
      }
    } catch (...) {
    }
    return fallback;
  }

  static std::string read_text_file_quiet(const fs::path & path)
  {
    std::ifstream input(path);
    if (!input) {
      return "";
    }
    std::ostringstream buffer;
    buffer << input.rdbuf();
    return buffer.str();
  }

  bool converted_map_ready(const std::string & map_name) const
  {
    const auto dir = converted_map_dir(map_name);
    if (dir.empty() || !fs::exists(dir / "map.json") || !fs::exists(dir / "map.pgm") ||
      !fs::exists(dir / "map.yaml"))
    {
      return false;
    }

    const std::string metadata = read_text_file_quiet(dir / "map.json");
    if (metadata.empty()) {
      return false;
    }
    const double min_z = get_parameter("converted_map_min_z").as_double();
    const double max_z = get_parameter("converted_map_max_z").as_double();
    const int min_component_cells = get_parameter("converted_map_min_component_cells").as_int();
    return text_contains_json_string(metadata, "projection_plane", "floor_plane") &&
      text_contains_json_bool(metadata, "floor_plane_used", true) &&
      text_contains_json_bool(metadata, "auto_floor_requested", true) &&
      text_contains_json_bool(metadata, "auto_floor_used", true) &&
      std::abs(metadata_number_value(metadata, "min_z_filter", -9999.0) - min_z) < 1e-6 &&
      std::abs(metadata_number_value(metadata, "max_z_filter", -9999.0) - max_z) < 1e-6 &&
      static_cast<int>(metadata_number_value(metadata, "min_component_cells", -1.0)) ==
        min_component_cells;
  }

  static uintmax_t file_size_or_zero(const fs::path & path)
  {
    try {
      return fs::exists(path) ? fs::file_size(path) : 0;
    } catch (...) {
      return 0;
    }
  }

  static std::vector<std::string> split_words(const std::string & line)
  {
    std::istringstream input(line);
    std::vector<std::string> words;
    std::string word;
    while (input >> word) {
      words.push_back(word);
    }
    return words;
  }

  static bool read_pcd_sample(const fs::path & path, const uint32_t max_points, CloudSnapshot & cloud)
  {
    std::ifstream input(path, std::ios::binary);
    if (!input) {
      return false;
    }

    std::vector<std::string> fields;
    std::vector<int> sizes;
    std::vector<int> counts;
    std::string data_type;
    uint64_t points = 0;
    std::string line;
    while (std::getline(input, line)) {
      if (!line.empty() && line.back() == '\r') {
        line.pop_back();
      }
      const auto words = split_words(line);
      if (words.empty()) {
        continue;
      }
      if (words[0] == "FIELDS") {
        fields.assign(words.begin() + 1, words.end());
      } else if (words[0] == "SIZE") {
        sizes.clear();
        for (size_t i = 1; i < words.size(); ++i) {
          sizes.push_back(std::stoi(words[i]));
        }
      } else if (words[0] == "COUNT") {
        counts.clear();
        for (size_t i = 1; i < words.size(); ++i) {
          counts.push_back(std::stoi(words[i]));
        }
      } else if (words[0] == "POINTS" && words.size() > 1) {
        points = static_cast<uint64_t>(std::stoull(words[1]));
      } else if (words[0] == "DATA" && words.size() > 1) {
        data_type = words[1];
        break;
      }
    }

    if (fields.empty() || sizes.empty() || points == 0 || data_type.empty()) {
      return false;
    }
    if (counts.empty()) {
      counts.assign(fields.size(), 1);
    }
    if (sizes.size() != fields.size() || counts.size() != fields.size()) {
      return false;
    }

    size_t point_step = 0;
    int x_offset = -1;
    int y_offset = -1;
    int z_offset = -1;
    for (size_t i = 0; i < fields.size(); ++i) {
      if (fields[i] == "x") {
        x_offset = static_cast<int>(point_step);
      } else if (fields[i] == "y") {
        y_offset = static_cast<int>(point_step);
      } else if (fields[i] == "z") {
        z_offset = static_cast<int>(point_step);
      }
      point_step += static_cast<size_t>(sizes[i]) * static_cast<size_t>(counts[i]);
    }
    if (x_offset < 0 || y_offset < 0 || z_offset < 0 || point_step == 0) {
      return false;
    }

    cloud.available = true;
    cloud.frame_id = "camera_init";
    cloud.source_frame_id = "camera_init";
    cloud.source_points = static_cast<uint32_t>(std::min<uint64_t>(
        points, std::numeric_limits<uint32_t>::max()));
    cloud.points.clear();
    cloud.points.reserve(static_cast<size_t>(std::min<uint64_t>(points, max_points)));
    const uint64_t stride = std::max<uint64_t>(
      1, (points + static_cast<uint64_t>(max_points) - 1) / static_cast<uint64_t>(max_points));

    if (data_type == "ascii") {
      std::string point_line;
      uint64_t index = 0;
      while (std::getline(input, point_line)) {
        if (index % stride != 0) {
          ++index;
          continue;
        }
        std::istringstream point_input(point_line);
        std::vector<float> values;
        float value = 0.0f;
        while (point_input >> value) {
          values.push_back(value);
        }
        if (values.size() >= 3) {
          const float x = values[0];
          const float y = values[1];
          const float z = values[2];
          if (std::isfinite(x) && std::isfinite(y) && std::isfinite(z)) {
            cloud.points.push_back({x, y, z});
          }
        }
        if (cloud.points.size() >= max_points) {
          break;
        }
        ++index;
      }
      return true;
    }

    if (data_type != "binary") {
      return false;
    }

    std::vector<char> buffer(point_step);
    for (uint64_t index = 0; index < points; ++index) {
      if (!input.read(buffer.data(), static_cast<std::streamsize>(point_step))) {
        break;
      }
      if (index % stride != 0 || cloud.points.size() >= max_points) {
        continue;
      }
      float x = 0.0f;
      float y = 0.0f;
      float z = 0.0f;
      std::memcpy(&x, buffer.data() + x_offset, sizeof(float));
      std::memcpy(&y, buffer.data() + y_offset, sizeof(float));
      std::memcpy(&z, buffer.data() + z_offset, sizeof(float));
      if (std::isfinite(x) && std::isfinite(y) && std::isfinite(z)) {
        cloud.points.push_back({x, y, z});
      }
    }
    return true;
  }

  static long long file_mtime_or_zero(const fs::path & path)
  {
    try {
      const auto ftime = fs::last_write_time(path);
      const auto sctp = std::chrono::time_point_cast<std::chrono::system_clock::duration>(
        ftime - fs::file_time_type::clock::now() + std::chrono::system_clock::now());
      return std::chrono::duration_cast<std::chrono::seconds>(sctp.time_since_epoch()).count();
    } catch (...) {
      return 0;
    }
  }

  std::string make_saved_maps_json()
  {
    struct Entry
    {
      std::string name;
      fs::path path;
      uintmax_t size = 0;
      long long mtime = 0;
      bool converted = false;
    };

    std::vector<Entry> entries;
    const auto dir = saved_maps_dir();
    if (fs::exists(dir)) {
      for (const auto & item : fs::directory_iterator(dir)) {
        if (!item.is_regular_file() || item.path().extension() != ".pcd") {
          continue;
        }
        Entry entry;
        entry.name = item.path().stem().string();
        entry.path = item.path();
        entry.size = file_size_or_zero(item.path());
        entry.mtime = file_mtime_or_zero(item.path());
        entry.converted = converted_map_ready(entry.name);
        entries.push_back(entry);
      }
    }
    std::sort(entries.begin(), entries.end(), [](const Entry & a, const Entry & b) {
      return a.mtime > b.mtime;
    });

    std::ostringstream json;
    json << "{";
    json << "\"ok\":true,";
    json << "\"maps_dir\":\"" << json_escape(dir.string()) << "\",";
    json << "\"convert_output_dir\":\"" << json_escape(pcd_to_pgm_output_dir().string()) << "\",";
    json << "\"maps\":[";
    for (size_t i = 0; i < entries.size(); ++i) {
      if (i > 0) {
        json << ",";
      }
      json << "{";
      json << "\"name\":\"" << json_escape(entries[i].name) << "\",";
      json << "\"pcd_path\":\"" << json_escape(entries[i].path.string()) << "\",";
      json << "\"size_bytes\":" << entries[i].size << ",";
      json << "\"mtime\":" << entries[i].mtime << ",";
      json << "\"converted\":" << (entries[i].converted ? "true" : "false") << ",";
      json << "\"converted_dir\":\"" << json_escape(converted_map_dir(entries[i].name).string()) << "\"";
      json << "}";
    }
    json << "]}";
    return json.str();
  }

  bool convert_saved_map(const std::string & map_name, std::string & response_json)
  {
    const std::string stem = sanitize_map_stem(map_name);
    const auto pcd = saved_map_pcd_path(stem);
    const auto output = converted_map_dir(stem);
    const auto binary = fs::path(get_parameter("pcd_to_pgm_binary").as_string());
    if (stem.empty() || !fs::exists(pcd)) {
      response_json = "{\"ok\":false,\"error\":\"PCD地图不存在\"}";
      return false;
    }
    if (!fs::exists(binary)) {
      response_json = "{\"ok\":false,\"error\":\"pcd2pgm转换工具不存在\",\"binary\":\"" +
        json_escape(binary.string()) + "\"}";
      return false;
    }
    if (converted_map_ready(stem)) {
      response_json = "{\"ok\":true,\"map_name\":\"" + json_escape(stem) +
        "\",\"converted\":true,\"skipped\":true,\"converted_dir\":\"" +
        json_escape(output.string()) + "\"}";
      return true;
    }

    fs::create_directories(output);
    std::string command;
    if (fs::exists("/usr/lib/aarch64-linux-gnu/libusb-1.0.so.0")) {
      command += "LD_PRELOAD=/usr/lib/aarch64-linux-gnu/libusb-1.0.so.0 ";
    }
    command += shell_quote(binary.string());
    command += " --input " + shell_quote(pcd.string());
    command += " --auto-floor-plane";
    FloorBasis expected_basis;
    {
      std::lock_guard<std::mutex> lock(map_mutex_);
      expected_basis = mapping_floor_basis_;
    }
    if (expected_basis.valid && expected_basis.source != "configured") {
      command += " --expected-floor-normal " + std::to_string(expected_basis.normal[0]) + " " +
        std::to_string(expected_basis.normal[1]) + " " + std::to_string(expected_basis.normal[2]);
    }
    const double min_z = get_parameter("converted_map_min_z").as_double();
    const double max_z = get_parameter("converted_map_max_z").as_double();
    const int min_component_cells = get_parameter("converted_map_min_component_cells").as_int();
    const double auto_floor_target_obstacle_ratio =
      get_parameter("converted_map_auto_floor_target_obstacle_ratio").as_double();
    const double auto_floor_max_obstacle_ratio =
      get_parameter("converted_map_auto_floor_max_obstacle_ratio").as_double();
    command += " --resolution 0.05 --voxel-leaf-size 0.05 --min-z " + std::to_string(min_z) +
      " --max-z " + std::to_string(max_z);
    command += " --auto-floor-target-obstacle-ratio " +
      std::to_string(auto_floor_target_obstacle_ratio);
    command += " --auto-floor-max-obstacle-ratio " +
      std::to_string(auto_floor_max_obstacle_ratio);
    command += " --min-component-cells " + std::to_string(min_component_cells) +
      " --inflate-radius 0.05";
    command += " --output " + shell_quote(output.string());

    const int rc = std::system(command.c_str());
    const bool ok = rc == 0 && converted_map_ready(stem);
    std::ostringstream json;
    json << "{";
    json << "\"ok\":" << (ok ? "true" : "false") << ",";
    json << "\"map_name\":\"" << json_escape(stem) << "\",";
    json << "\"converted\":" << (ok ? "true" : "false") << ",";
    json << "\"exit_code\":" << rc << ",";
    json << "\"pcd_path\":\"" << json_escape(pcd.string()) << "\",";
    json << "\"converted_dir\":\"" << json_escape(output.string()) << "\",";
    json << "\"floor_plane_source\":\"pcd2pgm_auto_floor\",";
    json << "\"floor_plane_inlier_ratio\":0.0";
    if (!ok) {
      json << ",\"error\":\"PCD转PGM失败\"";
    }
    json << "}";
    response_json = json.str();
    return ok;
  }

  static std::string read_text_file(const fs::path & path)
  {
    std::ifstream input(path);
    std::ostringstream buffer;
    buffer << input.rdbuf();
    return buffer.str();
  }

  static double json_number_value(
    const std::string & text, const std::string & key, const double fallback)
  {
    try {
      const std::regex pattern("\"" + key + "\"\\s*:\\s*(-?[0-9]+(\\.[0-9]+)?)");
      std::smatch match;
      if (std::regex_search(text, match, pattern)) {
        return std::stod(match[1].str());
      }
    } catch (...) {
    }
    return fallback;
  }

  static std::array<double, 3> json_origin_value(const std::string & text)
  {
    std::array<double, 3> origin{0.0, 0.0, 0.0};
    try {
      const std::regex pattern(
        "\"origin\"\\s*:\\s*\\[\\s*(-?[0-9]+(\\.[0-9]+)?)\\s*,\\s*(-?[0-9]+(\\.[0-9]+)?)\\s*,\\s*(-?[0-9]+(\\.[0-9]+)?)");
      std::smatch match;
      if (std::regex_search(text, match, pattern)) {
        origin[0] = std::stod(match[1].str());
        origin[1] = std::stod(match[3].str());
        origin[2] = std::stod(match[5].str());
      }
    } catch (...) {
    }
    return origin;
  }

  std::array<double, 4> json_floor_plane_value(const std::string & text) const
  {
    const auto fallback_param = get_parameter("floor_plane").as_double_array();
    std::array<double, 4> fallback{-0.357093, 0.006594, 0.934045, 1.001688};
    if (fallback_param.size() == 4) {
      fallback = {fallback_param[0], fallback_param[1], fallback_param[2], fallback_param[3]};
    }
    try {
      const std::regex pattern(
        "\"floor_plane\"\\s*:\\s*\\[\\s*(-?[0-9]+(\\.[0-9]+)?)\\s*,\\s*(-?[0-9]+(\\.[0-9]+)?)\\s*,\\s*(-?[0-9]+(\\.[0-9]+)?)\\s*,\\s*(-?[0-9]+(\\.[0-9]+)?)");
      std::smatch match;
      if (std::regex_search(text, match, pattern)) {
        return {
          std::stod(match[1].str()),
          std::stod(match[3].str()),
          std::stod(match[5].str()),
          std::stod(match[7].str())};
      }
    } catch (...) {
    }
    return fallback;
  }

  bool make_saved_map_view_json(const std::string & map_name, std::string & response_json)
  {
    std::string convert_json;
    if (!converted_map_ready(map_name) && !convert_saved_map(map_name, convert_json)) {
      response_json = convert_json;
      return false;
    }
    const std::string stem = sanitize_map_stem(map_name);
    const auto dir = converted_map_dir(stem);
    const auto metadata = read_text_file(dir / "map.json");
    const auto origin = json_origin_value(metadata);
    const int width = static_cast<int>(json_number_value(metadata, "width", 0));
    const int height = static_cast<int>(json_number_value(metadata, "height", 0));
    const double resolution = json_number_value(metadata, "resolution", 0.05);
    const auto metadata_basis = floor_basis_from_plane(json_floor_plane_value(metadata), "metadata");
    CloudSnapshot pcd_cloud;
    CloudSnapshot pcd_cloud_3d;
    read_pcd_sample(saved_map_pcd_path(stem), 15000, pcd_cloud);
    read_pcd_sample(
      saved_map_pcd_path(stem),
      static_cast<uint32_t>(std::max<int64_t>(15000, get_parameter("max_static_cloud_points").as_int())),
      pcd_cloud_3d);
    if (pcd_cloud_3d.available) {
      project_cloud_to_floor_frame(pcd_cloud_3d, metadata_basis);
    }
    if (get_parameter("project_saved_pcd_to_floor").as_bool()) {
      project_cloud_to_floor_frame(pcd_cloud, metadata_basis);
      filter_floor_height(
        pcd_cloud,
        get_parameter("converted_map_min_z").as_double(),
        get_parameter("converted_map_max_z").as_double());
    }

    std::ostringstream json;
    json << std::fixed << std::setprecision(3);
    json << "{";
    json << "\"ok\":true,";
    json << "\"map_name\":\"" << json_escape(stem) << "\",";
    json << "\"pcd_path\":\"" << json_escape(saved_map_pcd_path(stem).string()) << "\",";
    json << "\"converted_dir\":\"" << json_escape(dir.string()) << "\",";
    json << "\"yaml_path\":\"" << json_escape((dir / "map.yaml").string()) << "\",";
    json << "\"json_path\":\"" << json_escape((dir / "map.json").string()) << "\",";
    json << "\"preview_url\":\"/api/maps/preview.png?map_name=" << json_escape(stem) << "&t=0\",";
    json << "\"width\":" << width << ",";
    json << "\"height\":" << height << ",";
    json << "\"resolution\":" << resolution << ",";
    json << "\"origin\":{\"x\":" << origin[0] << ",\"y\":" << origin[1] << ",\"yaw\":" << origin[2] << "},";
    json << "\"pcd_cloud\":{";
    json << "\"available\":" << (pcd_cloud.available ? "true" : "false") << ",";
    json << "\"frame_id\":\"" << json_escape(pcd_cloud.frame_id) << "\",";
    json << "\"source_frame_id\":\"" << json_escape(pcd_cloud.source_frame_id) << "\",";
    json << "\"projected_to_floor\":" << (pcd_cloud.projected_to_floor ? "true" : "false") << ",";
    json << "\"floor_plane_source\":\"" << json_escape(pcd_cloud.floor_plane_source) << "\",";
    json << "\"floor_plane_inlier_ratio\":" << pcd_cloud.floor_plane_inlier_ratio << ",";
    json << "\"source_points\":" << pcd_cloud.source_points << ",";
    json << "\"points\":";
    append_points_json(json, pcd_cloud.points);
    json << "},";
    json << "\"pcd_cloud_3d\":{";
    json << "\"available\":" << (pcd_cloud_3d.available ? "true" : "false") << ",";
    json << "\"frame_id\":\"" << json_escape(pcd_cloud_3d.frame_id) << "\",";
    json << "\"source_frame_id\":\"" << json_escape(pcd_cloud_3d.source_frame_id) << "\",";
    json << "\"projected_to_floor\":" << (pcd_cloud_3d.projected_to_floor ? "true" : "false") << ",";
    json << "\"floor_plane_source\":\"" << json_escape(pcd_cloud_3d.floor_plane_source) << "\",";
    json << "\"floor_plane_inlier_ratio\":" << pcd_cloud_3d.floor_plane_inlier_ratio << ",";
    json << "\"source_points\":" << pcd_cloud_3d.source_points << ",";
    json << "\"points\":";
    append_points_json(json, pcd_cloud_3d.points);
    json << "},";
    json << "\"metadata\":" << (metadata.empty() ? "{}" : metadata);
    json << "}";
    response_json = json.str();
    return true;
  }

  bool make_saved_map_preview_png(
    const std::string & map_name, std::string & png, std::string & error)
  {
    std::string response_json;
    if (!converted_map_ready(map_name) && !convert_saved_map(map_name, response_json)) {
      error = "地图未转换且自动转换失败";
      return false;
    }
    const auto pgm = converted_map_dir(map_name) / "map.pgm";
    cv::Mat image = cv::imread(pgm.string(), cv::IMREAD_GRAYSCALE);
    if (image.empty()) {
      error = "无法读取PGM地图";
      return false;
    }
    std::vector<uchar> encoded;
    if (!cv::imencode(".png", image, encoded)) {
      error = "PNG编码失败";
      return false;
    }
    png.assign(reinterpret_cast<const char *>(encoded.data()), encoded.size());
    return true;
  }

  std::string make_mapping_status_json()
  {
    CloudSnapshot mapping_cloud;
    {
      std::lock_guard<std::mutex> lock(map_mutex_);
      mapping_cloud = mapping_cloud_;
    }

    bool active = false;
    bool last_save_success = false;
    std::string map_name;
    std::string message;
    std::string last_save_message;
    std::string last_save_path;
    {
      std::lock_guard<std::mutex> lock(mapping_mutex_);
      active = mapping_active_;
      map_name = mapping_map_name_;
      message = mapping_message_;
      last_save_success = last_save_success_;
      last_save_message = last_save_message_;
      last_save_path = last_save_path_;
    }

    const auto mapping_cloud_topic = get_parameter("mapping_cloud_topic").as_string();
    const auto save_service = get_parameter("map_save_service").as_string();
    std::ostringstream json;
    json << "{";
    json << "\"ok\":true,";
    json << "\"active\":" << (active ? "true" : "false") << ",";
    json << "\"map_name\":\"" << json_escape(map_name) << "\",";
    json << "\"message\":\"" << json_escape(message) << "\",";
    json << "\"mapping_cloud_topic\":\"" << json_escape(mapping_cloud_topic) << "\",";
    json << "\"mapping_cloud_publishers\":" << count_publishers(mapping_cloud_topic) << ",";
    json << "\"mapping_cloud_available\":" << (mapping_cloud.available ? "true" : "false") << ",";
    json << "\"mapping_cloud_points\":" << mapping_cloud.points.size() << ",";
    json << "\"mapping_cloud_source_points\":" << mapping_cloud.source_points << ",";
    json << "\"save_service\":\"" << json_escape(save_service) << "\",";
    json << "\"save_service_available\":" <<
      (map_save_client_ && map_save_client_->service_is_ready() ? "true" : "false") << ",";
    json << "\"last_save_success\":" << (last_save_success ? "true" : "false") << ",";
    json << "\"last_save_message\":\"" << json_escape(last_save_message) << "\",";
    json << "\"last_save_path\":\"" << json_escape(last_save_path) << "\"";
    json << "}";
    return json.str();
  }

  bool save_mapping_pcd(const std::string & map_name, std::string & response_json)
  {
    if (!map_save_client_) {
      response_json = "{\"ok\":false,\"error\":\"map save client is not initialized\"}";
      return false;
    }
    if (!map_save_client_->wait_for_service(std::chrono::seconds(2))) {
      std::lock_guard<std::mutex> lock(mapping_mutex_);
      last_save_success_ = false;
      last_save_message_ = "map save service is not available";
      last_save_path_.clear();
      response_json =
        "{\"ok\":false,\"error\":\"map save service is not available\",\"service\":\"" +
        json_escape(get_parameter("map_save_service").as_string()) + "\"}";
      return false;
    }

    auto request = std::make_shared<fast_lio::srv::SaveMap::Request>();
    request->map_name = map_name;
    auto future = map_save_client_->async_send_request(request);
    if (future.wait_for(std::chrono::seconds(10)) != std::future_status::ready) {
      std::lock_guard<std::mutex> lock(mapping_mutex_);
      last_save_success_ = false;
      last_save_message_ = "map save service timed out";
      last_save_path_.clear();
      response_json = "{\"ok\":false,\"error\":\"map save service timed out\"}";
      return false;
    }

    const auto result = future.get();
    {
      std::lock_guard<std::mutex> lock(mapping_mutex_);
      mapping_map_name_ = map_name;
      last_save_success_ = result->success;
      last_save_message_ = result->message;
      last_save_path_ = result->path;
      mapping_message_ = result->success ? "PCD地图保存成功。" : "PCD地图保存失败。";
    }

    std::ostringstream json;
    json << "{";
    json << "\"ok\":" << (result->success ? "true" : "false") << ",";
    json << "\"map_name\":\"" << json_escape(map_name) << "\",";
    json << "\"message\":\"" << json_escape(result->message) << "\",";
    json << "\"path\":\"" << json_escape(result->path) << "\"";
    json << "}";
    response_json = json.str();
    return result->success;
  }

  asio::io_context io_context_;
  std::unique_ptr<tcp::acceptor> acceptor_;
  std::thread server_thread_;
  std::atomic<bool> running_{false};
  std::mutex status_mutex_;
  std::mutex image_mutex_;
  std::mutex map_mutex_;
  std::mutex mapping_mutex_;
  std::string system_status_{"{}"};
  std::string control_status_{"{}"};
  std::string network_status_{"{}"};
  std::string latest_jpeg_;
  std::string latest_image_source_{"none"};
  std::atomic<bool> has_camera_frame_{false};
  std::mutex cmd_mutex_;
  std::string latest_cmd_session_;
  uint64_t latest_cmd_seq_{0};
  GridSnapshot grid_;
  GridSnapshot nav_costmap_;
  CloudSnapshot static_cloud_;
  CloudSnapshot live_cloud_;
  CloudSnapshot mapping_cloud_;
  CloudSnapshot static_cloud_3d_;
  CloudSnapshot live_cloud_3d_;
  CloudSnapshot mapping_cloud_3d_;
  FloorBasis mapping_floor_basis_;
  PoseSnapshot pose_;
  PoseSnapshot odom_pose_;
  PathSnapshot nav_path_;
  PathSnapshot nav_local_path_;
  std::string localization_status_;
  std::string nav_status_{"{}"};
  bool mapping_active_{false};
  bool mapping_start_time_valid_{false};
  rclcpp::Time mapping_started_at_{0, 0, RCL_ROS_TIME};
  bool last_save_success_{false};
  std::string mapping_map_name_;
  std::string mapping_message_{"建图传输未开启；网页不会接收FAST-LIO累计建图点云。"};
  std::string last_save_message_;
  std::string last_save_path_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr estop_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr nav_stop_pub_;
  rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr camera_set_parameters_client_;
  rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedPtr nav_get_parameters_client_;
  rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr nav_set_parameters_client_;
  rclcpp::Client<fast_lio::srv::SaveMap>::SharedPtr map_save_client_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_image_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr static_cloud_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr live_cloud_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr static_cloud_3d_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr live_cloud_3d_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr mapping_cloud_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr localization_status_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr nav_path_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr nav_local_path_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr nav_costmap_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr nav_cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr nav_status_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr system_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr control_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr network_sub_;
  std::atomic<bool> nav_drive_active_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WebApiNode>());
  rclcpp::shutdown();
  return 0;
}
