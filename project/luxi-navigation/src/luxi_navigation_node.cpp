#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <queue>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"

namespace
{

struct Cell
{
  int x{0};
  int y{0};

  bool operator==(const Cell & other) const
  {
    return x == other.x && y == other.y;
  }
};

struct QueueNode
{
  Cell cell;
  double f{0.0};
  double g{0.0};
};

struct QueueCompare
{
  bool operator()(const QueueNode & a, const QueueNode & b) const
  {
    return a.f > b.f;
  }
};

struct DwbSample
{
  double linear{0.0};
  double angular{0.0};
  double score{std::numeric_limits<double>::infinity()};
};

double yaw_from_quaternion(const geometry_msgs::msg::Quaternion & q)
{
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
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

class LuxiNavigationNode : public rclcpp::Node
{
public:
  LuxiNavigationNode()
  : Node("luxi_navigation_node")
  {
    declare_parameter<std::string>("map_topic", "/map");
    declare_parameter<std::string>("goal_topic", "/web/goal_pose");
    declare_parameter<std::string>("pose_topic", "/localization_2d");
    declare_parameter<std::string>("odom_topic", "/Odometry");
    declare_parameter<std::string>("path_topic", "/nav/path");
    declare_parameter<std::string>("local_path_topic", "/nav/local_path");
    declare_parameter<std::string>("costmap_topic", "/nav/costmap");
    declare_parameter<std::string>("status_topic", "/nav/status");
    declare_parameter<std::string>("stop_topic", "/nav/stop");
    declare_parameter<std::string>("frame_id", "map");
    declare_parameter<bool>("allow_odom_fallback", true);
    declare_parameter<int>("occupied_threshold", 65);
    declare_parameter<bool>("unknown_is_occupied", true);
    declare_parameter<double>("robot_radius_m", 0.28);
    declare_parameter<double>("extra_inflation_m", 0.10);
    declare_parameter<double>("start_snap_max_m", 2.0);
    declare_parameter<double>("goal_snap_max_m", 1.0);
    declare_parameter<int>("max_iterations", 900000);
    declare_parameter<double>("max_plan_time_sec", 3.0);
    declare_parameter<double>("simplify_epsilon_m", 0.08);
    declare_parameter<bool>("replan_on_goal", true);
    declare_parameter<bool>("replan_on_pose_update", false);
    declare_parameter<double>("min_replan_period_sec", 0.5);
    declare_parameter<bool>("controller_enabled", true);
    declare_parameter<std::string>("cmd_vel_topic", "/nav/cmd_vel");
    declare_parameter<double>("controller_frequency", 10.0);
    declare_parameter<double>("max_vel_x", 0.22);
    declare_parameter<double>("min_vel_x", 0.0);
    declare_parameter<double>("min_nonzero_vel_x", 0.08);
    declare_parameter<double>("max_vel_theta", 0.75);
    declare_parameter<int>("dwb_linear_samples", 6);
    declare_parameter<int>("dwb_angular_samples", 13);
    declare_parameter<double>("dwb_sim_time", 1.4);
    declare_parameter<double>("dwb_time_step", 0.1);
    declare_parameter<double>("dwb_path_distance_bias", 8.0);
    declare_parameter<double>("dwb_goal_distance_bias", 4.0);
    declare_parameter<double>("dwb_heading_bias", 1.5);
    declare_parameter<double>("dwb_velocity_bias", 0.4);
    declare_parameter<double>("dwb_stopped_trajectory_penalty", 2.0);
    declare_parameter<double>("dwb_collision_check_radius", 0.28);
    declare_parameter<double>("dwb_start_collision_grace_m", 0.25);
    declare_parameter<double>("goal_tolerance_xy", 0.18);
    declare_parameter<double>("goal_tolerance_yaw", 0.35);
    declare_parameter<double>("lookahead_distance", 0.60);

    const auto map_topic = get_parameter("map_topic").as_string();
    const auto goal_topic = get_parameter("goal_topic").as_string();
    const auto pose_topic = get_parameter("pose_topic").as_string();
    const auto odom_topic = get_parameter("odom_topic").as_string();
    const auto path_topic = get_parameter("path_topic").as_string();
    const auto local_path_topic = get_parameter("local_path_topic").as_string();
    const auto costmap_topic = get_parameter("costmap_topic").as_string();
    const auto status_topic = get_parameter("status_topic").as_string();
    const auto stop_topic = get_parameter("stop_topic").as_string();

    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      map_topic, rclcpp::QoS(1).transient_local().reliable(),
      std::bind(&LuxiNavigationNode::on_map, this, std::placeholders::_1));
    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      goal_topic, 10, std::bind(&LuxiNavigationNode::on_goal, this, std::placeholders::_1));
    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      pose_topic, 10, std::bind(&LuxiNavigationNode::on_pose, this, std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, 10, std::bind(&LuxiNavigationNode::on_odom, this, std::placeholders::_1));
    stop_sub_ = create_subscription<std_msgs::msg::Bool>(
      stop_topic, 10, std::bind(&LuxiNavigationNode::on_stop, this, std::placeholders::_1));

    path_pub_ = create_publisher<nav_msgs::msg::Path>(
      path_topic, rclcpp::QoS(1).transient_local().reliable());
    local_path_pub_ = create_publisher<nav_msgs::msg::Path>(
      local_path_topic, rclcpp::QoS(1).transient_local().reliable());
    costmap_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
      costmap_topic, rclcpp::QoS(1).transient_local().reliable());
    status_pub_ = create_publisher<std_msgs::msg::String>(
      status_topic, rclcpp::QoS(1).transient_local().reliable());
    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(
      get_parameter("cmd_vel_topic").as_string(), 10);

    const double controller_frequency =
      std::max(1.0, get_parameter("controller_frequency").as_double());
    controller_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / controller_frequency),
      std::bind(&LuxiNavigationNode::on_controller_timer, this));

    publish_status("waiting_map", "导航节点已启动，等待地图。");
    RCLCPP_INFO(
      get_logger(),
      "luxi_navigation_node started. map=%s goal=%s pose=%s odom=%s path=%s status=%s stop=%s",
      map_topic.c_str(), goal_topic.c_str(), pose_topic.c_str(), odom_topic.c_str(),
      path_topic.c_str(), status_topic.c_str(), stop_topic.c_str());
  }

private:
  void on_map(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    if (msg->info.width == 0 || msg->info.height == 0 || msg->info.resolution <= 0.0 ||
      msg->data.empty())
    {
      publish_status("map_invalid", "收到无效地图。");
      return;
    }

    map_ = *msg;
    map_ready_ = true;
    build_inflated_map();
    publish_costmap();
    publish_status("map_ready", "地图已加载，等待定位和目标点。");

    if (has_goal_ && has_pose() && get_parameter("replan_on_goal").as_bool()) {
      plan_and_publish();
    }
  }

  void on_goal(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    goal_ = *msg;
    has_goal_ = true;
    goal_reached_ = false;
    RCLCPP_INFO(
      get_logger(), "Goal received x=%.3f y=%.3f frame=%s",
      msg->pose.position.x, msg->pose.position.y, msg->header.frame_id.c_str());

    if (get_parameter("replan_on_goal").as_bool()) {
      plan_and_publish();
    }
  }

  void on_pose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    pose_.header = msg->header;
    pose_.pose = msg->pose;
    pose_.source = get_parameter("pose_topic").as_string();
    has_map_pose_ = true;
    maybe_replan_from_pose_update();
  }

  void on_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    odom_pose_.header = msg->header;
    odom_pose_.pose = msg->pose.pose;
    odom_pose_.source = get_parameter("odom_topic").as_string();
    last_odom_linear_x_ = msg->twist.twist.linear.x;
    last_odom_angular_z_ = msg->twist.twist.angular.z;
    has_odom_pose_ = true;
    maybe_replan_from_pose_update();
  }

  void on_stop(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (!msg->data) {
      return;
    }
    has_goal_ = false;
    goal_reached_ = false;
    publish_zero_cmd();
    publish_empty_path();
    publish_empty_local_path();
    publish_status("stopped", "收到停止导航命令，已清空全局/局部路径并发布零速度。");
  }

  void maybe_replan_from_pose_update()
  {
    if (!get_parameter("replan_on_pose_update").as_bool() || !has_goal_) {
      return;
    }
    const double min_period = get_parameter("min_replan_period_sec").as_double();
    if ((now() - last_plan_time_).seconds() < min_period) {
      return;
    }
    plan_and_publish();
  }

  bool has_pose() const
  {
    return has_map_pose_ ||
           (has_odom_pose_ && get_parameter("allow_odom_fallback").as_bool());
  }

  geometry_msgs::msg::PoseStamped current_pose() const
  {
    if (has_map_pose_) {
      return pose_;
    }
    return odom_pose_;
  }

  void build_inflated_map()
  {
    const int width = static_cast<int>(map_.info.width);
    const int height = static_cast<int>(map_.info.height);
    inflated_occupied_.assign(static_cast<size_t>(width * height), 0);
    inflated_cost_values_.assign(static_cast<size_t>(width * height), 0);

    const double radius =
      std::max(0.0, get_parameter("robot_radius_m").as_double()) +
      std::max(0.0, get_parameter("extra_inflation_m").as_double());
    const int inflation_cells =
      std::max(0, static_cast<int>(std::ceil(radius / map_.info.resolution)));
    const int threshold = get_parameter("occupied_threshold").as_int();
    const bool unknown_occupied = get_parameter("unknown_is_occupied").as_bool();

    auto mark = [&](const int x, const int y) {
      if (x < 0 || y < 0 || x >= width || y >= height) {
        return;
      }
      const auto idx = static_cast<size_t>(y * width + x);
      inflated_occupied_[idx] = 1;
      inflated_cost_values_[idx] = std::max<int8_t>(inflated_cost_values_[idx], 70);
    };

    for (int y = 0; y < height; ++y) {
      for (int x = 0; x < width; ++x) {
        const int value = map_.data[static_cast<size_t>(y * width + x)];
        const bool occupied = value >= threshold || (value < 0 && unknown_occupied);
        if (!occupied) {
          continue;
        }
        const auto obstacle_idx = static_cast<size_t>(y * width + x);
        inflated_occupied_[obstacle_idx] = 1;
        inflated_cost_values_[obstacle_idx] = 100;
        for (int dy = -inflation_cells; dy <= inflation_cells; ++dy) {
          for (int dx = -inflation_cells; dx <= inflation_cells; ++dx) {
            if ((dx * dx + dy * dy) >
              (inflation_cells * inflation_cells))
            {
              continue;
            }
            mark(x + dx, y + dy);
          }
        }
        inflated_cost_values_[obstacle_idx] = 100;
      }
    }
  }

  void publish_costmap()
  {
    if (!map_ready_ || inflated_cost_values_.empty()) {
      return;
    }
    nav_msgs::msg::OccupancyGrid costmap;
    costmap.header.stamp = now();
    costmap.header.frame_id = get_parameter("frame_id").as_string();
    costmap.info = map_.info;
    costmap.data = inflated_cost_values_;
    costmap_pub_->publish(costmap);
  }

  bool world_to_cell(const double wx, const double wy, Cell & cell) const
  {
    const double ox = map_.info.origin.position.x;
    const double oy = map_.info.origin.position.y;
    const double yaw = yaw_from_quaternion(map_.info.origin.orientation);
    const double dx = wx - ox;
    const double dy = wy - oy;
    const double c = std::cos(-yaw);
    const double s = std::sin(-yaw);
    const double mx = c * dx - s * dy;
    const double my = s * dx + c * dy;
    cell.x = static_cast<int>(std::floor(mx / map_.info.resolution));
    cell.y = static_cast<int>(std::floor(my / map_.info.resolution));
    return is_inside(cell);
  }

  geometry_msgs::msg::PoseStamped cell_to_pose(const Cell & cell, const double yaw) const
  {
    const double local_x = (static_cast<double>(cell.x) + 0.5) * map_.info.resolution;
    const double local_y = (static_cast<double>(cell.y) + 0.5) * map_.info.resolution;
    const double origin_yaw = yaw_from_quaternion(map_.info.origin.orientation);
    const double c = std::cos(origin_yaw);
    const double s = std::sin(origin_yaw);

    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = now();
    pose.header.frame_id = get_parameter("frame_id").as_string();
    pose.pose.position.x = map_.info.origin.position.x + c * local_x - s * local_y;
    pose.pose.position.y = map_.info.origin.position.y + s * local_x + c * local_y;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.z = std::sin(yaw * 0.5);
    pose.pose.orientation.w = std::cos(yaw * 0.5);
    return pose;
  }

  int index_of(const Cell & cell) const
  {
    return cell.y * static_cast<int>(map_.info.width) + cell.x;
  }

  bool is_inside(const Cell & cell) const
  {
    return cell.x >= 0 && cell.y >= 0 &&
           cell.x < static_cast<int>(map_.info.width) &&
           cell.y < static_cast<int>(map_.info.height);
  }

  bool is_free(const Cell & cell) const
  {
    if (!is_inside(cell)) {
      return false;
    }
    const auto idx = static_cast<size_t>(index_of(cell));
    return idx < inflated_occupied_.size() && inflated_occupied_[idx] == 0;
  }

  bool snap_to_nearest_free(const Cell & seed, const double max_distance_m, Cell & result) const
  {
    if (is_free(seed)) {
      result = seed;
      return true;
    }
    const int max_radius = std::max(
      1, static_cast<int>(std::ceil(std::max(0.0, max_distance_m) / map_.info.resolution)));
    for (int r = 1; r <= max_radius; ++r) {
      for (int dy = -r; dy <= r; ++dy) {
        for (int dx = -r; dx <= r; ++dx) {
          if (std::max(std::abs(dx), std::abs(dy)) != r) {
            continue;
          }
          Cell candidate{seed.x + dx, seed.y + dy};
          if (is_free(candidate)) {
            result = candidate;
            return true;
          }
        }
      }
    }
    return false;
  }

  int raw_map_value(const Cell & cell) const
  {
    if (!is_inside(cell)) {
      return -999;
    }
    return map_.data[static_cast<size_t>(index_of(cell))];
  }

  double heuristic(const Cell & a, const Cell & b) const
  {
    const double dx = static_cast<double>(a.x - b.x);
    const double dy = static_cast<double>(a.y - b.y);
    return std::sqrt(dx * dx + dy * dy);
  }

  bool plan_cells(const Cell & start, const Cell & goal, std::vector<Cell> & path)
  {
    const int width = static_cast<int>(map_.info.width);
    const int height = static_cast<int>(map_.info.height);
    const int total = width * height;
    const int max_iterations = static_cast<int>(
      std::max<int64_t>(1, get_parameter("max_iterations").as_int()));
    const double max_plan_time = std::max(0.1, get_parameter("max_plan_time_sec").as_double());
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(max_plan_time);

    std::vector<float> g_score(static_cast<size_t>(total), std::numeric_limits<float>::infinity());
    std::vector<int> parent(static_cast<size_t>(total), -1);
    std::vector<uint8_t> closed(static_cast<size_t>(total), 0);
    std::priority_queue<QueueNode, std::vector<QueueNode>, QueueCompare> open;

    const int start_idx = index_of(start);
    const int goal_idx = index_of(goal);
    g_score[static_cast<size_t>(start_idx)] = 0.0F;
    open.push(QueueNode{start, heuristic(start, goal), 0.0});

    const std::array<Cell, 8> dirs{
      Cell{1, 0}, Cell{-1, 0}, Cell{0, 1}, Cell{0, -1},
      Cell{1, 1}, Cell{1, -1}, Cell{-1, 1}, Cell{-1, -1}};

    int iterations = 0;
    while (!open.empty() && iterations < max_iterations) {
      if (std::chrono::steady_clock::now() > deadline) {
        publish_status("plan_timeout", "规划超时。");
        return false;
      }
      ++iterations;
      const auto current = open.top();
      open.pop();
      const int current_idx = index_of(current.cell);
      if (closed[static_cast<size_t>(current_idx)] != 0) {
        continue;
      }
      closed[static_cast<size_t>(current_idx)] = 1;
      if (current_idx == goal_idx) {
        reconstruct_path(parent, start_idx, goal_idx, path);
        RCLCPP_INFO(get_logger(), "A* success iterations=%d cells=%zu", iterations, path.size());
        return true;
      }

      for (const auto & dir : dirs) {
        const Cell next{current.cell.x + dir.x, current.cell.y + dir.y};
        if (!is_free(next)) {
          continue;
        }
        if (dir.x != 0 && dir.y != 0) {
          const Cell side_a{current.cell.x + dir.x, current.cell.y};
          const Cell side_b{current.cell.x, current.cell.y + dir.y};
          if (!is_free(side_a) || !is_free(side_b)) {
            continue;
          }
        }
        const int next_idx = index_of(next);
        if (closed[static_cast<size_t>(next_idx)] != 0) {
          continue;
        }
        const double step = (dir.x != 0 && dir.y != 0) ? std::sqrt(2.0) : 1.0;
        const double tentative = current.g + step;
        if (tentative < g_score[static_cast<size_t>(next_idx)]) {
          g_score[static_cast<size_t>(next_idx)] = static_cast<float>(tentative);
          parent[static_cast<size_t>(next_idx)] = current_idx;
          open.push(QueueNode{next, tentative + heuristic(next, goal), tentative});
        }
      }
    }

    publish_status("plan_failed", "未找到可行路径。");
    return false;
  }

  void reconstruct_path(
    const std::vector<int> & parent, const int start_idx, const int goal_idx,
    std::vector<Cell> & path) const
  {
    path.clear();
    const int width = static_cast<int>(map_.info.width);
    int idx = goal_idx;
    while (idx >= 0) {
      path.push_back(Cell{idx % width, idx / width});
      if (idx == start_idx) {
        break;
      }
      idx = parent[static_cast<size_t>(idx)];
    }
    std::reverse(path.begin(), path.end());
  }

  bool is_line_free(const Cell & a, const Cell & b) const
  {
    int x0 = a.x;
    int y0 = a.y;
    const int x1 = b.x;
    const int y1 = b.y;
    const int dx = std::abs(x1 - x0);
    const int dy = -std::abs(y1 - y0);
    const int sx = x0 < x1 ? 1 : -1;
    const int sy = y0 < y1 ? 1 : -1;
    int err = dx + dy;
    while (true) {
      if (!is_free(Cell{x0, y0})) {
        return false;
      }
      if (x0 == x1 && y0 == y1) {
        return true;
      }
      const int e2 = 2 * err;
      if (e2 >= dy) {
        err += dy;
        x0 += sx;
      }
      if (e2 <= dx) {
        err += dx;
        y0 += sy;
      }
    }
  }

  std::vector<Cell> simplify_path(const std::vector<Cell> & raw) const
  {
    if (raw.size() <= 2) {
      return raw;
    }
    std::vector<Cell> simplified;
    simplified.push_back(raw.front());
    size_t anchor = 0;
    while (anchor + 1 < raw.size()) {
      size_t best = anchor + 1;
      for (size_t i = raw.size() - 1; i > anchor + 1; --i) {
        if (is_line_free(raw[anchor], raw[i])) {
          best = i;
          break;
        }
      }
      simplified.push_back(raw[best]);
      anchor = best;
    }
    return simplified;
  }

  bool plan_and_publish()
  {
    if (!map_ready_) {
      publish_status("waiting_map", "等待 /map。");
      current_path_.poses.clear();
      return false;
    }
    if (!has_pose()) {
      publish_status("waiting_pose", "等待 /localization_2d 或 /Odometry。");
      current_path_.poses.clear();
      return false;
    }
    if (!has_goal_) {
      publish_status("waiting_goal", "等待网页目标点。");
      current_path_.poses.clear();
      return false;
    }

    const auto start_pose = current_pose();
    Cell start_raw;
    Cell goal_raw;
    if (!world_to_cell(start_pose.pose.position.x, start_pose.pose.position.y, start_raw)) {
      publish_status("start_outside_map", "机器人当前位置不在地图范围内。");
      current_path_.poses.clear();
      return false;
    }
    if (!world_to_cell(goal_.pose.position.x, goal_.pose.position.y, goal_raw)) {
      publish_status("goal_outside_map", "目标点不在地图范围内。");
      current_path_.poses.clear();
      return false;
    }

    Cell start;
    Cell goal;
    if (!snap_to_nearest_free(start_raw, get_parameter("start_snap_max_m").as_double(), start)) {
      std::ostringstream detail;
      detail << "起点附近没有可通行栅格 raw=(" << start_raw.x << "," << start_raw.y
             << ") value=" << raw_map_value(start_raw)
             << " snap_max_m=" << get_parameter("start_snap_max_m").as_double();
      publish_status("start_blocked", detail.str());
      current_path_.poses.clear();
      return false;
    }
    if (!snap_to_nearest_free(goal_raw, get_parameter("goal_snap_max_m").as_double(), goal)) {
      std::ostringstream detail;
      detail << "目标点附近没有可通行栅格 raw=(" << goal_raw.x << "," << goal_raw.y
             << ") value=" << raw_map_value(goal_raw)
             << " snap_max_m=" << get_parameter("goal_snap_max_m").as_double();
      publish_status("goal_blocked", detail.str());
      current_path_.poses.clear();
      return false;
    }

    publish_status("planning", "正在规划全局路径。");
    std::vector<Cell> raw_path;
    last_plan_time_ = now();
    if (!plan_cells(start, goal, raw_path)) {
      publish_empty_path();
      return false;
    }

    const auto simplified = simplify_path(raw_path);
    publish_path(simplified);

    std::ostringstream detail;
    detail << "规划成功 raw_cells=" << raw_path.size() << " waypoints=" << simplified.size()
           << " start=(" << start.x << "," << start.y << ") goal=(" << goal.x << "," << goal.y
           << ")";
    publish_status("path_ready", detail.str());
    return true;
  }

  void publish_empty_path()
  {
    nav_msgs::msg::Path path;
    path.header.stamp = now();
    path.header.frame_id = get_parameter("frame_id").as_string();
    path_pub_->publish(path);
    current_path_ = path;
  }

  void publish_path(const std::vector<Cell> & cells)
  {
    nav_msgs::msg::Path path;
    path.header.stamp = now();
    path.header.frame_id = get_parameter("frame_id").as_string();
    const auto robot = current_pose();

    if (cells.size() <= 1) {
      auto start = robot;
      start.header = path.header;
      const double yaw = std::atan2(
        goal_.pose.position.y - start.pose.position.y,
        goal_.pose.position.x - start.pose.position.x);
      start.pose.orientation.z = std::sin(yaw * 0.5);
      start.pose.orientation.w = std::cos(yaw * 0.5);
      path.poses.push_back(start);
      if (distance2d(start, goal_) > map_.info.resolution) {
        path.poses.push_back(goal_);
        path.poses.back().header = path.header;
      }
      path_pub_->publish(path);
      current_path_ = path;
      goal_reached_ = false;
      return;
    }

    for (size_t i = 0; i < cells.size(); ++i) {
      double yaw = 0.0;
      if (i + 1 < cells.size()) {
        const auto p0 = cell_to_pose(cells[i], 0.0);
        const auto p1 = cell_to_pose(cells[i + 1], 0.0);
        yaw = std::atan2(
          p1.pose.position.y - p0.pose.position.y,
          p1.pose.position.x - p0.pose.position.x);
      } else {
        yaw = yaw_from_quaternion(goal_.pose.orientation);
      }
      path.poses.push_back(cell_to_pose(cells[i], yaw));
    }
    if (path.poses.empty() || distance2d(robot, path.poses.front()) > map_.info.resolution) {
      auto start = robot;
      start.header = path.header;
      if (!path.poses.empty()) {
        const double yaw = std::atan2(
          path.poses.front().pose.position.y - start.pose.position.y,
          path.poses.front().pose.position.x - start.pose.position.x);
        start.pose.orientation.z = std::sin(yaw * 0.5);
        start.pose.orientation.w = std::cos(yaw * 0.5);
      }
      path.poses.insert(path.poses.begin(), start);
    }
    if (path.poses.empty() || distance2d(path.poses.back(), goal_) > map_.info.resolution) {
      if (!path.poses.empty()) {
        const auto & last = path.poses.back();
        const double yaw = std::atan2(
          goal_.pose.position.y - last.pose.position.y,
          goal_.pose.position.x - last.pose.position.x);
        path.poses.back().pose.orientation.z = std::sin(yaw * 0.5);
        path.poses.back().pose.orientation.w = std::cos(yaw * 0.5);
      }
      path.poses.push_back(goal_);
      path.poses.back().header = path.header;
    }

    path_pub_->publish(path);
    current_path_ = path;
    goal_reached_ = false;
  }

  static double normalize_angle(double angle)
  {
    while (angle > M_PI) {
      angle -= 2.0 * M_PI;
    }
    while (angle < -M_PI) {
      angle += 2.0 * M_PI;
    }
    return angle;
  }

  static double distance2d(
    const geometry_msgs::msg::PoseStamped & a,
    const geometry_msgs::msg::PoseStamped & b)
  {
    const double dx = a.pose.position.x - b.pose.position.x;
    const double dy = a.pose.position.y - b.pose.position.y;
    return std::hypot(dx, dy);
  }

  double distance_to_path(const double x, const double y) const
  {
    if (current_path_.poses.empty()) {
      return 0.0;
    }
    double best = std::numeric_limits<double>::infinity();
    for (const auto & pose : current_path_.poses) {
      best = std::min(best, std::hypot(x - pose.pose.position.x, y - pose.pose.position.y));
    }
    return best;
  }

  geometry_msgs::msg::PoseStamped lookahead_pose(const geometry_msgs::msg::PoseStamped & robot) const
  {
    if (current_path_.poses.empty()) {
      return goal_;
    }
    const double lookahead = std::max(0.05, get_parameter("lookahead_distance").as_double());
    size_t nearest = 0;
    double nearest_dist = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < current_path_.poses.size(); ++i) {
      const double dist = distance2d(robot, current_path_.poses[i]);
      if (dist < nearest_dist) {
        nearest = i;
        nearest_dist = dist;
      }
    }

    double accumulated = 0.0;
    for (size_t i = nearest + 1; i < current_path_.poses.size(); ++i) {
      accumulated += distance2d(current_path_.poses[i - 1], current_path_.poses[i]);
      if (accumulated >= lookahead) {
        return current_path_.poses[i];
      }
    }
    return current_path_.poses.back();
  }

  bool pose_is_free(const double x, const double y, const double radius) const
  {
    Cell center;
    if (!world_to_cell(x, y, center)) {
      return false;
    }
    const int radius_cells = std::max(0, static_cast<int>(std::ceil(radius / map_.info.resolution)));
    for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
      for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
        if (dx * dx + dy * dy > radius_cells * radius_cells) {
          continue;
        }
        if (!is_free(Cell{center.x + dx, center.y + dy})) {
          return false;
        }
      }
    }
    return true;
  }

  bool score_dwb_sample(
    const geometry_msgs::msg::PoseStamped & robot, const double linear,
    const double angular, DwbSample & sample) const
  {
    const double sim_time = std::max(0.2, get_parameter("dwb_sim_time").as_double());
    const double dt = std::clamp(get_parameter("dwb_time_step").as_double(), 0.02, 0.5);
    const double collision_radius =
      std::max(0.0, get_parameter("dwb_collision_check_radius").as_double());

    double x = robot.pose.position.x;
    double y = robot.pose.position.y;
    const double start_x = x;
    const double start_y = y;
    double yaw = yaw_from_quaternion(robot.pose.orientation);
    for (double t = 0.0; t < sim_time; t += dt) {
      x += linear * std::cos(yaw) * dt;
      y += linear * std::sin(yaw) * dt;
      yaw = normalize_angle(yaw + angular * dt);
      const double dist_from_start = std::hypot(x - start_x, y - start_y);
      const double grace = std::max(0.0, get_parameter("dwb_start_collision_grace_m").as_double());
      if (dist_from_start > grace && !pose_is_free(x, y, collision_radius)) {
        return false;
      }
    }

    const auto target = lookahead_pose(robot);
    const double path_dist = distance_to_path(x, y);
    const double goal_dist = std::hypot(x - goal_.pose.position.x, y - goal_.pose.position.y);
    const double target_yaw = std::atan2(target.pose.position.y - y, target.pose.position.x - x);
    const double heading_error = std::abs(normalize_angle(target_yaw - yaw));
    sample.linear = linear;
    sample.angular = angular;
    sample.score =
      get_parameter("dwb_path_distance_bias").as_double() * path_dist +
      get_parameter("dwb_goal_distance_bias").as_double() * goal_dist +
      get_parameter("dwb_heading_bias").as_double() * heading_error -
      get_parameter("dwb_velocity_bias").as_double() * linear;
    if (std::abs(linear) < 1e-6) {
      sample.score += get_parameter("dwb_stopped_trajectory_penalty").as_double();
    }
    return true;
  }

  bool compute_dwb_command(const geometry_msgs::msg::PoseStamped & robot, geometry_msgs::msg::Twist & cmd)
  {
    const int linear_samples = std::max<int64_t>(2, get_parameter("dwb_linear_samples").as_int());
    const int angular_samples = std::max<int64_t>(3, get_parameter("dwb_angular_samples").as_int());
    const double max_v = std::max(0.0, get_parameter("max_vel_x").as_double());
    const double min_positive_v = std::clamp(
      get_parameter("min_nonzero_vel_x").as_double(), 0.0, max_v);
    const double max_w = std::max(0.05, get_parameter("max_vel_theta").as_double());

    DwbSample best;
    bool found = false;
    for (int i = 0; i < linear_samples; ++i) {
      double linear = 0.0;
      if (i > 0 && max_v > 0.0) {
        const double ratio = linear_samples > 2 ?
          static_cast<double>(i - 1) / static_cast<double>(linear_samples - 2) : 1.0;
        linear = min_positive_v + (max_v - min_positive_v) * ratio;
      }
      for (int j = 0; j < angular_samples; ++j) {
        const double ar = static_cast<double>(j) / static_cast<double>(angular_samples - 1);
        const double angular = -max_w + 2.0 * max_w * ar;
        DwbSample sample;
        if (!score_dwb_sample(robot, linear, angular, sample)) {
          continue;
        }
        if (!found || sample.score < best.score) {
          best = sample;
          found = true;
        }
      }
    }

    if (!found) {
      publish_empty_local_path();
      return false;
    }
    cmd.linear.x = best.linear;
    cmd.angular.z = best.angular;
    publish_local_path(robot, best.linear, best.angular);
    return true;
  }

  void publish_empty_local_path()
  {
    nav_msgs::msg::Path path;
    path.header.stamp = now();
    path.header.frame_id = get_parameter("frame_id").as_string();
    local_path_pub_->publish(path);
  }

  void publish_local_path(
    const geometry_msgs::msg::PoseStamped & robot, const double linear, const double angular)
  {
    nav_msgs::msg::Path path;
    path.header.stamp = now();
    path.header.frame_id = get_parameter("frame_id").as_string();

    const double sim_time = std::max(0.2, get_parameter("dwb_sim_time").as_double());
    const double dt = std::clamp(get_parameter("dwb_time_step").as_double(), 0.02, 0.5);
    double x = robot.pose.position.x;
    double y = robot.pose.position.y;
    double yaw = yaw_from_quaternion(robot.pose.orientation);
    for (double t = 0.0; t <= sim_time; t += dt) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position.x = x;
      pose.pose.position.y = y;
      pose.pose.position.z = 0.03;
      pose.pose.orientation.z = std::sin(yaw * 0.5);
      pose.pose.orientation.w = std::cos(yaw * 0.5);
      path.poses.push_back(pose);
      x += linear * std::cos(yaw) * dt;
      y += linear * std::sin(yaw) * dt;
      yaw = normalize_angle(yaw + angular * dt);
    }
    local_path_pub_->publish(path);
  }

  void publish_zero_cmd()
  {
    geometry_msgs::msg::Twist cmd;
    cmd_pub_->publish(cmd);
  }

  void on_controller_timer()
  {
    if (!get_parameter("controller_enabled").as_bool()) {
      return;
    }
    if (!map_ready_ || !has_pose() || !has_goal_ || current_path_.poses.empty()) {
      publish_empty_local_path();
      return;
    }

    const auto robot = current_pose();
    const double goal_dist =
      std::hypot(robot.pose.position.x - goal_.pose.position.x, robot.pose.position.y - goal_.pose.position.y);
    const double yaw_error = std::abs(normalize_angle(
      yaw_from_quaternion(goal_.pose.orientation) - yaw_from_quaternion(robot.pose.orientation)));
    if (goal_dist <= get_parameter("goal_tolerance_xy").as_double() &&
      yaw_error <= get_parameter("goal_tolerance_yaw").as_double())
    {
      if (!goal_reached_) {
        publish_zero_cmd();
        publish_empty_local_path();
        goal_reached_ = true;
        publish_status("goal_reached", "DWB已到达目标容差范围，发布零速度。");
      }
      return;
    }

    geometry_msgs::msg::Twist cmd;
    if (!compute_dwb_command(robot, cmd)) {
      publish_zero_cmd();
      publish_empty_local_path();
      publish_status("controller_blocked", "DWB未找到无碰撞局部轨迹，发布零速度。");
      return;
    }
    cmd_pub_->publish(cmd);
  }

  void publish_status(const std::string & state, const std::string & detail)
  {
    std_msgs::msg::String msg;
    std::ostringstream json;
    json << "{";
    json << "\"state\":\"" << json_escape(state) << "\",";
    json << "\"detail\":\"" << json_escape(detail) << "\",";
    json << "\"map_ready\":" << (map_ready_ ? "true" : "false") << ",";
    json << "\"has_map_pose\":" << (has_map_pose_ ? "true" : "false") << ",";
    json << "\"has_odom_pose\":" << (has_odom_pose_ ? "true" : "false") << ",";
    json << "\"has_goal\":" << (has_goal_ ? "true" : "false") << ",";
    json << "\"has_path\":" << (!current_path_.poses.empty() ? "true" : "false") << ",";
    json << "\"controller_enabled\":" <<
      (get_parameter("controller_enabled").as_bool() ? "true" : "false") << ",";
    json << "\"cmd_vel_topic\":\"" << json_escape(get_parameter("cmd_vel_topic").as_string()) << "\",";
    json << "\"path_topic\":\"" << json_escape(get_parameter("path_topic").as_string()) << "\"";
    json << "}";
    msg.data = json.str();
    status_pub_->publish(msg);
  }

  struct PoseWithSource : public geometry_msgs::msg::PoseStamped
  {
    std::string source;
  };

  nav_msgs::msg::OccupancyGrid map_;
  std::vector<uint8_t> inflated_occupied_;
  std::vector<int8_t> inflated_cost_values_;
  bool map_ready_{false};
  bool has_goal_{false};
  bool has_map_pose_{false};
  bool has_odom_pose_{false};
  rclcpp::Time last_plan_time_{0, 0, RCL_ROS_TIME};
  geometry_msgs::msg::PoseStamped goal_;
  nav_msgs::msg::Path current_path_;
  PoseWithSource pose_;
  PoseWithSource odom_pose_;
  bool goal_reached_{false};
  double last_odom_linear_x_{0.0};
  double last_odom_angular_z_{0.0};

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr stop_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr local_path_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr controller_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LuxiNavigationNode>());
  rclcpp::shutdown();
  return 0;
}
