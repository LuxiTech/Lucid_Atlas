#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Dense>

#include <algorithm>
#include <array>
#include <cmath>
#include <cctype>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <queue>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

namespace {

constexpr std::uint8_t kOccupiedValue = 0;
constexpr std::uint8_t kFreeValue = 254;
constexpr std::uint8_t kUnknownValue = 205;

using Cloud = pcl::PointCloud<pcl::PointXYZ>;

struct Config {
  std::string input;
  std::string output_dir;
  std::string output_prefix;
  std::string output_pgm;
  std::string output_yaml;
  std::string metadata;
  double resolution = 0.05;
  double voxel_leaf_size = 0.0;
  double min_z = -std::numeric_limits<double>::infinity();
  double max_z = std::numeric_limits<double>::infinity();
  char height_axis = 'z';
  bool use_floor_plane = false;
  std::array<double, 3> floor_normal{0.0, 0.0, 1.0};
  double floor_d = 0.0;
  bool auto_floor_plane = false;
  bool has_expected_floor_normal = false;
  std::array<double, 3> expected_floor_normal{0.0, 0.0, 1.0};
  double auto_floor_target_obstacle_ratio = 0.06;
  double auto_floor_min_obstacle_ratio = 0.01;
  double auto_floor_max_obstacle_ratio = 0.18;
  double auto_floor_max_below_ratio = 0.08;
  double inflate_radius = 0.0;
  int min_component_cells = 0;
  int padding_cells = 1;
  bool unknown_as_free = true;
  bool radius_outlier = false;
  double radius_search = 0.1;
  int min_neighbors = 10;
  double occupied_threshold = 0.65;
  double free_threshold = 0.196;
  bool use_transform = false;
  double tx = 0.0;
  double ty = 0.0;
  double tz = 0.0;
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
};

struct Bounds {
  double min_x = std::numeric_limits<double>::infinity();
  double min_y = std::numeric_limits<double>::infinity();
  double max_x = -std::numeric_limits<double>::infinity();
  double max_y = -std::numeric_limits<double>::infinity();

  bool valid() const {
    return std::isfinite(min_x) && std::isfinite(min_y) && std::isfinite(max_x) && std::isfinite(max_y);
  }
};

struct Stats {
  std::size_t raw_points = 0;
  std::size_t transformed_points = 0;
  std::size_t voxel_points = 0;
  std::size_t height_filtered_points = 0;
  std::size_t radius_filtered_points = 0;
  std::size_t occupied_cells_before_inflation = 0;
  std::size_t occupied_cells_after_component_filter = 0;
  std::size_t occupied_cells_after_inflation = 0;
  std::size_t removed_components = 0;
  std::size_t removed_component_cells = 0;
  std::size_t free_cells = 0;
  std::size_t unknown_cells = 0;
  int width = 0;
  int height = 0;
  double origin_x = 0.0;
  double origin_y = 0.0;
  double min_z_seen = std::numeric_limits<double>::infinity();
  double max_z_seen = -std::numeric_limits<double>::infinity();
  std::string projection_plane;
  Bounds projected_bounds;
  bool auto_floor_requested = false;
  bool auto_floor_used = false;
  std::string auto_floor_status;
  double auto_floor_score = 0.0;
  double auto_floor_obstacle_ratio = 0.0;
  double auto_floor_below_ratio = 0.0;
  double auto_floor_near_ratio = 0.0;
  double auto_floor_angle_to_expected_deg = 0.0;
  double auto_floor_quantile = 0.0;
  std::array<double, 3> auto_floor_expected_normal{0.0, 0.0, 1.0};
  bool auto_floor_normal_fit_used = false;
  double auto_floor_normal_inlier_ratio = 0.0;
};

void print_usage(const char *program) {
  std::cout
      << "Usage:\n"
      << "  " << program << " --input map.pcd --output ./maps [options]\n\n"
      << "Options:\n"
      << "  --input PATH                 Input PCD file\n"
      << "  --output DIR                 Output directory. Writes map.pgm, map.yaml, map.json\n"
      << "  --output-prefix PATH         Output path without extension. Default: output/<input-stem>\n"
      << "  --pgm PATH                   Output PGM path. Overrides --output-prefix for image only\n"
      << "  --yaml PATH                  Output YAML path. Overrides --output-prefix for yaml only\n"
      << "  --metadata PATH              Output JSON metadata path. Default: <output-prefix>.json\n"
      << "  --resolution M               Output map resolution in meters, default 0.05\n"
      << "  --voxel-leaf-size M          Optional PCD downsample size. 0 disables it\n"
      << "  --min-z M                    Minimum point height to project\n"
      << "  --max-z M                    Maximum point height to project\n"
      << "  --height-axis x|y|z          Axis used as height before projection. Default: z\n"
      << "  --floor-plane NX NY NZ D     Use plane NX*x+NY*y+NZ*z+D=0 as floor; min/max are height above it\n"
      << "  --auto-floor-plane           Estimate floor plane offset from the PCD before conversion\n"
      << "  --expected-floor-normal X Y Z Expected floor normal from IMU/live floor estimate; strongly recommended\n"
      << "  --auto-floor-target-obstacle-ratio R  Preferred ratio of points in min/max obstacle band. Default: 0.06\n"
      << "  --auto-floor-min-obstacle-ratio R     Reject very sparse obstacle bands. Default: 0.01\n"
      << "  --auto-floor-max-obstacle-ratio R     Reject dense obstacle bands. Default: 0.18\n"
      << "  --auto-floor-max-below-ratio R        Reject planes with too many below-floor points. Default: 0.08\n"
      << "  --radius-outlier             Enable radius outlier filtering\n"
      << "  --radius-search M            Radius outlier search radius, default 0.10\n"
      << "  --min-neighbors N            Radius outlier minimum neighbors, default 10\n"
      << "  --inflate-radius M           Inflate occupied cells by this radius in meters. Default: 0\n"
      << "  --min-component-cells N      Remove occupied connected components smaller than N cells\n"
      << "  --padding-cells N            Padding around projected map. Default: 1\n"
      << "  --keep-unknown               Keep non-obstacle cells unknown. Default writes them as free\n"
      << "  --occupied-threshold P       YAML occupied threshold, default 0.65\n"
      << "  --free-threshold P           YAML free threshold, default 0.196\n"
      << "  --transform X Y Z R P Y      Apply translation and roll/pitch/yaw radians before projection\n"
      << "  --help                       Show this help\n\n"
      << "Examples:\n"
      << "  " << program << " --input map.pcd --resolution 0.05 --min-z -0.2 --max-z 1.2 --output ./map_nav\n"
      << "  " << program << " --input map.pcd --height-axis y --min-z -0.2 --max-z 1.2 --output ./map_nav\n"
      << "  " << program << " --input map.pcd --floor-plane -0.436 0.208 0.876 1.243 --min-z 0.15 --max-z 1.6 --output ./map_nav\n"
      << "  " << program << " --input map.pcd --auto-floor-plane --expected-floor-normal -0.34 -0.02 0.94 --min-z 0.05 --max-z 0.50 --output ./map_nav\n"
      << "  " << program << " --input map.pcd --radius-outlier --radius-search 0.1 --min-neighbors 10 --output ./map_nav\n";
}

double parse_double(const std::string &value, const std::string &name) {
  char *end = nullptr;
  const double parsed = std::strtod(value.c_str(), &end);
  if (end == value.c_str() || *end != '\0') {
    throw std::runtime_error("Invalid value for " + name + ": " + value);
  }
  return parsed;
}

int parse_int(const std::string &value, const std::string &name) {
  char *end = nullptr;
  const long parsed = std::strtol(value.c_str(), &end, 10);
  if (end == value.c_str() || *end != '\0') {
    throw std::runtime_error("Invalid value for " + name + ": " + value);
  }
  if (parsed < std::numeric_limits<int>::min() || parsed > std::numeric_limits<int>::max()) {
    throw std::runtime_error("Value out of range for " + name + ": " + value);
  }
  return static_cast<int>(parsed);
}

char parse_axis(const std::string &value, const std::string &name) {
  if (value.size() != 1) {
    throw std::runtime_error("Invalid value for " + name + ": " + value);
  }
  const char axis = static_cast<char>(std::tolower(value[0]));
  if (axis != 'x' && axis != 'y' && axis != 'z') {
    throw std::runtime_error("Invalid value for " + name + ": " + value);
  }
  return axis;
}

Config parse_args(int argc, char **argv) {
  Config config;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    auto need_value = [&](const std::string &name) -> std::string {
      if (i + 1 >= argc) {
        throw std::runtime_error("Missing value for " + name);
      }
      return argv[++i];
    };

    if (arg == "--help" || arg == "-h") {
      print_usage(argv[0]);
      std::exit(0);
    } else if (arg == "--input") {
      config.input = need_value(arg);
    } else if (arg == "--output") {
      config.output_dir = need_value(arg);
    } else if (arg == "--output-prefix") {
      config.output_prefix = need_value(arg);
    } else if (arg == "--pgm") {
      config.output_pgm = need_value(arg);
    } else if (arg == "--yaml") {
      config.output_yaml = need_value(arg);
    } else if (arg == "--metadata") {
      config.metadata = need_value(arg);
    } else if (arg == "--resolution") {
      config.resolution = parse_double(need_value(arg), arg);
    } else if (arg == "--voxel-leaf-size") {
      config.voxel_leaf_size = parse_double(need_value(arg), arg);
    } else if (arg == "--min-z") {
      config.min_z = parse_double(need_value(arg), arg);
    } else if (arg == "--max-z") {
      config.max_z = parse_double(need_value(arg), arg);
    } else if (arg == "--height-axis") {
      config.height_axis = parse_axis(need_value(arg), arg);
    } else if (arg == "--floor-plane") {
      if (i + 4 >= argc) {
        throw std::runtime_error("Missing values for --floor-plane NX NY NZ D");
      }
      config.floor_normal[0] = parse_double(argv[++i], "--floor-plane NX");
      config.floor_normal[1] = parse_double(argv[++i], "--floor-plane NY");
      config.floor_normal[2] = parse_double(argv[++i], "--floor-plane NZ");
      config.floor_d = parse_double(argv[++i], "--floor-plane D");
      config.use_floor_plane = true;
    } else if (arg == "--auto-floor-plane") {
      config.auto_floor_plane = true;
    } else if (arg == "--expected-floor-normal") {
      if (i + 3 >= argc) {
        throw std::runtime_error("Missing values for --expected-floor-normal X Y Z");
      }
      config.expected_floor_normal[0] = parse_double(argv[++i], "--expected-floor-normal X");
      config.expected_floor_normal[1] = parse_double(argv[++i], "--expected-floor-normal Y");
      config.expected_floor_normal[2] = parse_double(argv[++i], "--expected-floor-normal Z");
      config.has_expected_floor_normal = true;
    } else if (arg == "--auto-floor-target-obstacle-ratio") {
      config.auto_floor_target_obstacle_ratio = parse_double(need_value(arg), arg);
    } else if (arg == "--auto-floor-min-obstacle-ratio") {
      config.auto_floor_min_obstacle_ratio = parse_double(need_value(arg), arg);
    } else if (arg == "--auto-floor-max-obstacle-ratio") {
      config.auto_floor_max_obstacle_ratio = parse_double(need_value(arg), arg);
    } else if (arg == "--auto-floor-max-below-ratio") {
      config.auto_floor_max_below_ratio = parse_double(need_value(arg), arg);
    } else if (arg == "--radius-outlier") {
      config.radius_outlier = true;
    } else if (arg == "--radius-search") {
      config.radius_search = parse_double(need_value(arg), arg);
    } else if (arg == "--min-neighbors") {
      config.min_neighbors = parse_int(need_value(arg), arg);
    } else if (arg == "--inflate-radius") {
      config.inflate_radius = parse_double(need_value(arg), arg);
    } else if (arg == "--min-component-cells") {
      config.min_component_cells = parse_int(need_value(arg), arg);
    } else if (arg == "--padding-cells") {
      config.padding_cells = parse_int(need_value(arg), arg);
    } else if (arg == "--keep-unknown") {
      config.unknown_as_free = false;
    } else if (arg == "--occupied-threshold") {
      config.occupied_threshold = parse_double(need_value(arg), arg);
    } else if (arg == "--free-threshold") {
      config.free_threshold = parse_double(need_value(arg), arg);
    } else if (arg == "--transform") {
      if (i + 6 >= argc) {
        throw std::runtime_error("Missing values for --transform X Y Z R P Y");
      }
      config.tx = parse_double(argv[++i], "--transform X");
      config.ty = parse_double(argv[++i], "--transform Y");
      config.tz = parse_double(argv[++i], "--transform Z");
      config.roll = parse_double(argv[++i], "--transform R");
      config.pitch = parse_double(argv[++i], "--transform P");
      config.yaw = parse_double(argv[++i], "--transform Y");
      config.use_transform = true;
    } else {
      throw std::runtime_error("Unknown argument: " + arg);
    }
  }

  if (config.input.empty()) {
    throw std::runtime_error("--input is required");
  }
  if (config.resolution <= 0.0) {
    throw std::runtime_error("--resolution must be > 0");
  }
  if (config.voxel_leaf_size < 0.0) {
    throw std::runtime_error("--voxel-leaf-size must be >= 0");
  }
  if (config.min_z > config.max_z) {
    throw std::runtime_error("--min-z must be <= --max-z");
  }
  if (config.auto_floor_plane) {
    if (!std::isfinite(config.min_z) || !std::isfinite(config.max_z)) {
      throw std::runtime_error("--auto-floor-plane requires finite --min-z and --max-z");
    }
    if (config.auto_floor_target_obstacle_ratio <= 0.0 || config.auto_floor_target_obstacle_ratio >= 1.0) {
      throw std::runtime_error("--auto-floor-target-obstacle-ratio must be between 0 and 1");
    }
    if (config.auto_floor_min_obstacle_ratio < 0.0 || config.auto_floor_min_obstacle_ratio >= 1.0) {
      throw std::runtime_error("--auto-floor-min-obstacle-ratio must be in [0, 1)");
    }
    if (config.auto_floor_max_obstacle_ratio <= 0.0 || config.auto_floor_max_obstacle_ratio > 1.0) {
      throw std::runtime_error("--auto-floor-max-obstacle-ratio must be in (0, 1]");
    }
    if (config.auto_floor_min_obstacle_ratio > config.auto_floor_max_obstacle_ratio) {
      throw std::runtime_error("--auto-floor-min-obstacle-ratio must be <= --auto-floor-max-obstacle-ratio");
    }
    if (config.auto_floor_max_below_ratio < 0.0 || config.auto_floor_max_below_ratio > 1.0) {
      throw std::runtime_error("--auto-floor-max-below-ratio must be between 0 and 1");
    }
  }
  if (config.has_expected_floor_normal) {
    const double norm = std::sqrt(config.expected_floor_normal[0] * config.expected_floor_normal[0] +
                                  config.expected_floor_normal[1] * config.expected_floor_normal[1] +
                                  config.expected_floor_normal[2] * config.expected_floor_normal[2]);
    if (norm <= 0.0) {
      throw std::runtime_error("--expected-floor-normal must be non-zero");
    }
    config.expected_floor_normal[0] /= norm;
    config.expected_floor_normal[1] /= norm;
    config.expected_floor_normal[2] /= norm;
    if (config.expected_floor_normal[2] < 0.0) {
      config.expected_floor_normal[0] = -config.expected_floor_normal[0];
      config.expected_floor_normal[1] = -config.expected_floor_normal[1];
      config.expected_floor_normal[2] = -config.expected_floor_normal[2];
    }
  }
  if (config.use_floor_plane) {
    const double norm = std::sqrt(config.floor_normal[0] * config.floor_normal[0] +
                                  config.floor_normal[1] * config.floor_normal[1] +
                                  config.floor_normal[2] * config.floor_normal[2]);
    if (norm <= 0.0) {
      throw std::runtime_error("--floor-plane normal must be non-zero");
    }
    config.floor_normal[0] /= norm;
    config.floor_normal[1] /= norm;
    config.floor_normal[2] /= norm;
    config.floor_d /= norm;
    if (config.floor_normal[2] < 0.0) {
      config.floor_normal[0] = -config.floor_normal[0];
      config.floor_normal[1] = -config.floor_normal[1];
      config.floor_normal[2] = -config.floor_normal[2];
      config.floor_d = -config.floor_d;
    }
  }
  if (config.radius_search <= 0.0) {
    throw std::runtime_error("--radius-search must be > 0");
  }
  if (config.min_neighbors < 0) {
    throw std::runtime_error("--min-neighbors must be >= 0");
  }
  if (config.inflate_radius < 0.0) {
    throw std::runtime_error("--inflate-radius must be >= 0");
  }
  if (config.min_component_cells < 0) {
    throw std::runtime_error("--min-component-cells must be >= 0");
  }
  if (config.padding_cells < 0) {
    throw std::runtime_error("--padding-cells must be >= 0");
  }
  if (config.occupied_threshold <= 0.0 || config.occupied_threshold >= 1.0) {
    throw std::runtime_error("--occupied-threshold must be between 0 and 1");
  }
  if (config.free_threshold <= 0.0 || config.free_threshold >= 1.0) {
    throw std::runtime_error("--free-threshold must be between 0 and 1");
  }
  if (config.free_threshold >= config.occupied_threshold) {
    throw std::runtime_error("--free-threshold must be less than --occupied-threshold");
  }

  if (!config.output_dir.empty()) {
    const std::filesystem::path output_dir(config.output_dir);
    if (config.output_pgm.empty()) {
      config.output_pgm = (output_dir / "map.pgm").string();
    }
    if (config.output_yaml.empty()) {
      config.output_yaml = (output_dir / "map.yaml").string();
    }
    if (config.metadata.empty()) {
      config.metadata = (output_dir / "map.json").string();
    }
  } else {
    if (config.output_prefix.empty()) {
      const std::filesystem::path input_path(config.input);
      config.output_prefix = (std::filesystem::path("output") / input_path.stem()).string();
    }
    if (config.output_pgm.empty()) {
      config.output_pgm = config.output_prefix + ".pgm";
    }
    if (config.output_yaml.empty()) {
      config.output_yaml = config.output_prefix + ".yaml";
    }
    if (config.metadata.empty()) {
      config.metadata = config.output_prefix + ".json";
    }
  }

  return config;
}

void ensure_parent_directory(const std::string &path) {
  const std::filesystem::path file_path(path);
  const std::filesystem::path parent = file_path.parent_path();
  if (!parent.empty()) {
    std::filesystem::create_directories(parent);
  }
}

Cloud::Ptr load_pcd(const std::string &path) {
  Cloud::Ptr cloud(new Cloud());
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(path, *cloud) != 0) {
    throw std::runtime_error("Failed to read PCD: " + path);
  }
  return cloud;
}

Cloud::Ptr transform_cloud(const Cloud::Ptr &input, const Config &config) {
  Cloud::Ptr output(new Cloud());
  output->reserve(input->size());

  const double cr = std::cos(config.roll);
  const double sr = std::sin(config.roll);
  const double cp = std::cos(config.pitch);
  const double sp = std::sin(config.pitch);
  const double cy = std::cos(config.yaw);
  const double sy = std::sin(config.yaw);

  const double r00 = cy * cp;
  const double r01 = cy * sp * sr - sy * cr;
  const double r02 = cy * sp * cr + sy * sr;
  const double r10 = sy * cp;
  const double r11 = sy * sp * sr + cy * cr;
  const double r12 = sy * sp * cr - cy * sr;
  const double r20 = -sp;
  const double r21 = cp * sr;
  const double r22 = cp * cr;

  for (const auto &point : input->points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
      continue;
    }

    pcl::PointXYZ out;
    if (config.use_transform) {
      out.x = static_cast<float>(r00 * point.x + r01 * point.y + r02 * point.z + config.tx);
      out.y = static_cast<float>(r10 * point.x + r11 * point.y + r12 * point.z + config.ty);
      out.z = static_cast<float>(r20 * point.x + r21 * point.y + r22 * point.z + config.tz);
    } else {
      out = point;
    }
    output->push_back(out);
  }

  output->width = static_cast<std::uint32_t>(output->size());
  output->height = 1;
  output->is_dense = false;
  return output;
}

Cloud::Ptr voxel_downsample(const Cloud::Ptr &input, double leaf_size) {
  if (leaf_size <= 0.0) {
    return input;
  }

  pcl::VoxelGrid<pcl::PointXYZ> filter;
  filter.setInputCloud(input);
  filter.setLeafSize(static_cast<float>(leaf_size), static_cast<float>(leaf_size), static_cast<float>(leaf_size));

  Cloud::Ptr output(new Cloud());
  filter.filter(*output);
  return output;
}

Cloud::Ptr filter_height(const Cloud::Ptr &input, double min_z, double max_z, Stats &stats) {
  Cloud::Ptr output(new Cloud());
  output->reserve(input->size());

  for (const auto &point : input->points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
      continue;
    }
    stats.min_z_seen = std::min(stats.min_z_seen, static_cast<double>(point.z));
    stats.max_z_seen = std::max(stats.max_z_seen, static_cast<double>(point.z));
    if (point.z < min_z || point.z > max_z) {
      continue;
    }
    output->push_back(point);
  }

  output->width = static_cast<std::uint32_t>(output->size());
  output->height = 1;
  output->is_dense = false;
  return output;
}

double axis_value(const pcl::PointXYZ &point, char axis) {
  if (axis == 'x') return point.x;
  if (axis == 'y') return point.y;
  return point.z;
}

double height_value(const pcl::PointXYZ &point, const Config &config) {
  if (config.use_floor_plane) {
    return config.floor_normal[0] * point.x + config.floor_normal[1] * point.y +
           config.floor_normal[2] * point.z + config.floor_d;
  }
  return axis_value(point, config.height_axis);
}

void floor_plane_basis(const Config &config, std::array<double, 3> &u, std::array<double, 3> &v) {
  const std::array<double, 3> n = config.floor_normal;
  std::array<double, 3> seed{1.0, 0.0, 0.0};
  if (std::abs(n[0]) > 0.9) {
    seed = {0.0, 1.0, 0.0};
  }

  const double dot = seed[0] * n[0] + seed[1] * n[1] + seed[2] * n[2];
  u = {seed[0] - dot * n[0], seed[1] - dot * n[1], seed[2] - dot * n[2]};
  const double u_norm = std::sqrt(u[0] * u[0] + u[1] * u[1] + u[2] * u[2]);
  u = {u[0] / u_norm, u[1] / u_norm, u[2] / u_norm};

  v = {n[1] * u[2] - n[2] * u[1],
       n[2] * u[0] - n[0] * u[2],
       n[0] * u[1] - n[1] * u[0]};
  const double v_norm = std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
  v = {v[0] / v_norm, v[1] / v_norm, v[2] / v_norm};
}

std::pair<double, double> projected_values(const pcl::PointXYZ &point, const Config &config) {
  if (config.use_floor_plane) {
    std::array<double, 3> u;
    std::array<double, 3> v;
    floor_plane_basis(config, u, v);
    return {
        point.x * u[0] + point.y * u[1] + point.z * u[2],
        point.x * v[0] + point.y * v[1] + point.z * v[2],
    };
  }

  const char height_axis = config.height_axis;
  if (height_axis == 'x') {
    return {point.y, point.z};
  }
  if (height_axis == 'y') {
    return {point.x, point.z};
  }
  return {point.x, point.y};
}

std::string projection_plane_for_axis(char height_axis) {
  if (height_axis == 'x') return "yz";
  if (height_axis == 'y') return "xz";
  return "xy";
}

std::string projection_plane_for_config(const Config &config) {
  if (config.use_floor_plane) {
    return "floor_plane";
  }
  return projection_plane_for_axis(config.height_axis);
}

Cloud::Ptr filter_height_axis(const Cloud::Ptr &input, const Config &config, Stats &stats) {
  Cloud::Ptr output(new Cloud());
  output->reserve(input->size());

  for (const auto &point : input->points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
      continue;
    }
    const double height = height_value(point, config);
    stats.min_z_seen = std::min(stats.min_z_seen, height);
    stats.max_z_seen = std::max(stats.max_z_seen, height);
    if (height < config.min_z || height > config.max_z) {
      continue;
    }
    output->push_back(point);
  }

  output->width = static_cast<std::uint32_t>(output->size());
  output->height = 1;
  output->is_dense = false;
  return output;
}

Cloud::Ptr filter_radius_outliers(const Cloud::Ptr &input, const Config &config) {
  if (!config.radius_outlier || input->empty()) {
    return input;
  }

  pcl::RadiusOutlierRemoval<pcl::PointXYZ> filter;
  filter.setInputCloud(input);
  filter.setRadiusSearch(config.radius_search);
  filter.setMinNeighborsInRadius(config.min_neighbors);

  Cloud::Ptr output(new Cloud());
  filter.filter(*output);
  return output;
}

double dot3(const std::array<double, 3> &a, const std::array<double, 3> &b) {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

bool normalize3(std::array<double, 3> &value) {
  const double norm = std::sqrt(dot3(value, value));
  if (norm < 1e-12) {
    return false;
  }
  value[0] /= norm;
  value[1] /= norm;
  value[2] /= norm;
  if (value[2] < 0.0) {
    value[0] = -value[0];
    value[1] = -value[1];
    value[2] = -value[2];
  }
  return true;
}

double angle_degrees_between_normals(std::array<double, 3> a, std::array<double, 3> b) {
  if (!normalize3(a) || !normalize3(b)) {
    return 0.0;
  }
  const double cosine = std::clamp(std::abs(dot3(a, b)), -1.0, 1.0);
  return std::acos(cosine) * 180.0 / 3.14159265358979323846;
}

class FloorPlaneAnalyzer {
public:
  explicit FloorPlaneAnalyzer(const Config &config) : config_(config) {}

  bool apply(Cloud::Ptr cloud, Config &config, Stats &stats) const {
    stats.auto_floor_requested = config_.auto_floor_plane;
    if (!config_.auto_floor_plane) {
      stats.auto_floor_status = "disabled";
      return true;
    }
    if (!cloud || cloud->empty()) {
      stats.auto_floor_status = "empty_cloud";
      return false;
    }

    std::array<double, 3> reference_normal = expected_normal(config);
    if (!normalize3(reference_normal)) {
      stats.auto_floor_status = "invalid_reference_normal";
      return false;
    }

    stats.auto_floor_expected_normal = reference_normal;

    EvaluatedPlane best_plane = evaluate_plane(
        cloud, reference_normal, reference_normal, false, 0.0, !config.has_expected_floor_normal);
    if (!config.has_expected_floor_normal) {
      const NormalFit full_fit = fit_floor_normal(cloud, reference_normal, false);
      if (full_fit.valid) {
        const EvaluatedPlane evaluated =
            evaluate_plane(cloud, full_fit.normal, reference_normal, true, full_fit.inlier_ratio, true);
        if (evaluated.valid && (!best_plane.valid || evaluated.candidate.score > best_plane.candidate.score)) {
          best_plane = evaluated;
        }
      }
      const NormalFit low_band_fit = fit_floor_normal(cloud, reference_normal, true);
      if (low_band_fit.valid) {
        const EvaluatedPlane evaluated =
            evaluate_plane(cloud, low_band_fit.normal, reference_normal, true, low_band_fit.inlier_ratio, true);
        if (evaluated.valid && (!best_plane.valid || evaluated.candidate.score > best_plane.candidate.score)) {
          best_plane = evaluated;
        }
      }
    }
    const NormalFit lower_envelope_fit = fit_lower_envelope_floor(cloud, reference_normal);
    if (lower_envelope_fit.valid &&
        (!config.has_expected_floor_normal || lower_envelope_fit.angle_to_reference_deg > 3.0)) {
      const EvaluatedPlane evaluated = evaluate_fixed_plane(
          cloud, lower_envelope_fit.normal, lower_envelope_fit.d, true,
          lower_envelope_fit.inlier_ratio);
      const bool stronger_ground_support =
          best_plane.valid &&
          evaluated.candidate.floor_support_ratio > best_plane.candidate.floor_support_ratio + 0.015 &&
          evaluated.candidate.below_ratio + 0.01 < best_plane.candidate.below_ratio;
      if (evaluated.valid &&
          (!best_plane.valid || evaluated.candidate.score > best_plane.candidate.score - 0.25 ||
           (stronger_ground_support && evaluated.candidate.score > best_plane.candidate.score - 0.75))) {
        best_plane = evaluated;
      }
    }

    if (!best_plane.valid) {
      stats.auto_floor_status = "no_valid_candidate";
      return false;
    }

    config.use_floor_plane = true;
    config.floor_normal = best_plane.normal;
    config.floor_d = best_plane.candidate.d;
    stats.auto_floor_used = true;
    stats.auto_floor_status = "selected";
    stats.auto_floor_score = best_plane.candidate.score;
    stats.auto_floor_obstacle_ratio = best_plane.candidate.obstacle_ratio;
    stats.auto_floor_below_ratio = best_plane.candidate.below_ratio;
    stats.auto_floor_near_ratio = best_plane.candidate.near_ratio;
    stats.auto_floor_angle_to_expected_deg =
        angle_degrees_between_normals(best_plane.normal, stats.auto_floor_expected_normal);
    stats.auto_floor_quantile = best_plane.candidate.quantile;
    stats.auto_floor_normal_fit_used = best_plane.normal_fit_used;
    stats.auto_floor_normal_inlier_ratio = best_plane.normal_inlier_ratio;
    return true;
  }

private:
  struct Candidate {
    double quantile = 0.0;
    double d = 0.0;
    double score = -1.0;
    double obstacle_ratio = 0.0;
    double below_ratio = 0.0;
    double near_ratio = 0.0;
    double floor_support_ratio = 0.0;
  };

  struct NormalFit {
    bool valid = false;
    std::array<double, 3> normal{0.0, 0.0, 1.0};
    double d = 0.0;
    double inlier_ratio = 0.0;
    double angle_to_reference_deg = 0.0;
    double score = -std::numeric_limits<double>::infinity();
  };

  struct EvaluatedPlane {
    bool valid = false;
    std::array<double, 3> normal{0.0, 0.0, 1.0};
    Candidate candidate;
    bool normal_fit_used = false;
    double normal_inlier_ratio = 0.0;
  };

  std::array<double, 3> expected_normal(const Config &config) const {
    if (config.has_expected_floor_normal) {
      return config.expected_floor_normal;
    }
    if (config.use_floor_plane) {
      return config.floor_normal;
    }
    return {0.0, 0.0, 1.0};
  }

  static bool plane_from_points(const pcl::PointXYZ &a, const pcl::PointXYZ &b, const pcl::PointXYZ &c,
                                std::array<double, 3> &normal, double &d) {
    const std::array<double, 3> ab{
        static_cast<double>(b.x - a.x),
        static_cast<double>(b.y - a.y),
        static_cast<double>(b.z - a.z)};
    const std::array<double, 3> ac{
        static_cast<double>(c.x - a.x),
        static_cast<double>(c.y - a.y),
        static_cast<double>(c.z - a.z)};
    normal = {
        ab[1] * ac[2] - ab[2] * ac[1],
        ab[2] * ac[0] - ab[0] * ac[2],
        ab[0] * ac[1] - ab[1] * ac[0],
    };
    if (!normalize3(normal)) {
      return false;
    }
    d = -(normal[0] * a.x + normal[1] * a.y + normal[2] * a.z);
    return true;
  }

  std::vector<double> signed_distances_for_normal(const Cloud::Ptr &cloud, const std::array<double, 3> &normal) const {
    std::vector<double> signed_distances;
    signed_distances.reserve(cloud->size());
    for (const auto &point : cloud->points) {
      if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
        continue;
      }
      signed_distances.push_back(normal[0] * point.x + normal[1] * point.y + normal[2] * point.z);
    }
    std::sort(signed_distances.begin(), signed_distances.end());
    return signed_distances;
  }

  Candidate select_floor_offset(const std::vector<double> &signed_distances) const {
    const std::array<double, 16> quantiles{
      0.005, 0.01, 0.02, 0.035, 0.05, 0.07, 0.08, 0.10,
      0.12, 0.15, 0.18, 0.20, 0.25, 0.30, 0.35, 0.40};
    Candidate best;
    for (const double quantile : quantiles) {
      const size_t index = std::min(
          signed_distances.size() - 1,
          static_cast<size_t>(std::floor(quantile * static_cast<double>(signed_distances.size() - 1))));
      Candidate candidate;
      candidate.quantile = quantile;
      candidate.d = -signed_distances[index];
      score_candidate(signed_distances, candidate);
      if (candidate.score > best.score) {
        best = candidate;
      }
    }
    return best;
  }

  EvaluatedPlane evaluate_plane(const Cloud::Ptr &cloud, const std::array<double, 3> &input_normal,
                                const std::array<double, 3> &reference_normal, bool normal_fit_used,
                                double normal_inlier_ratio, bool allow_refine) const {
    EvaluatedPlane evaluated;
    std::array<double, 3> normal = input_normal;
    if (!normalize3(normal)) {
      return evaluated;
    }

    std::vector<double> signed_distances = signed_distances_for_normal(cloud, normal);
    if (signed_distances.size() < 100) {
      return evaluated;
    }

    Candidate candidate = select_floor_offset(signed_distances);
    if (allow_refine && candidate.score >= 0.0) {
      const NormalFit refined_fit = refine_normal_from_floor_band(cloud, normal, candidate.d, reference_normal);
      if (refined_fit.valid) {
        normal = refined_fit.normal;
        signed_distances = signed_distances_for_normal(cloud, normal);
        candidate = select_floor_offset(signed_distances);
        normal_fit_used = true;
        normal_inlier_ratio = refined_fit.inlier_ratio;
      }
    }
    if (candidate.score < 0.0) {
      return evaluated;
    }

    evaluated.valid = true;
    evaluated.normal = normal;
    evaluated.candidate = candidate;
    evaluated.normal_fit_used = normal_fit_used;
    evaluated.normal_inlier_ratio = normal_inlier_ratio;
    return evaluated;
  }

  EvaluatedPlane evaluate_fixed_plane(const Cloud::Ptr &cloud, const std::array<double, 3> &input_normal,
                                      double d, bool normal_fit_used, double normal_inlier_ratio) const {
    EvaluatedPlane evaluated;
    std::array<double, 3> normal = input_normal;
    if (!normalize3(normal)) {
      return evaluated;
    }

    std::vector<double> signed_distances = signed_distances_for_normal(cloud, normal);
    if (signed_distances.size() < 100) {
      return evaluated;
    }

    Candidate candidate;
    candidate.quantile = -1.0;
    candidate.d = d;
    score_candidate(signed_distances, candidate);
    if (candidate.score < 0.0) {
      return evaluated;
    }

    evaluated.valid = true;
    evaluated.normal = normal;
    evaluated.candidate = candidate;
    evaluated.normal_fit_used = normal_fit_used;
    evaluated.normal_inlier_ratio = normal_inlier_ratio;
    return evaluated;
  }

  NormalFit fit_floor_normal(const Cloud::Ptr &cloud, const std::array<double, 3> &reference_normal,
                             bool use_low_height_band) const {
    NormalFit best;
    if (!cloud || cloud->size() < 100) {
      return best;
    }

    std::vector<const pcl::PointXYZ *> sample;
    sample.reserve(std::min<std::size_t>(cloud->size(), 12000));
    const std::size_t stride = std::max<std::size_t>(1, cloud->size() / 12000);
    for (std::size_t i = 0; i < cloud->size(); i += stride) {
      const auto &point = cloud->points[i];
      if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z)) {
        sample.push_back(&point);
      }
    }
    if (sample.size() < 100) {
      return best;
    }
    const double inlier_threshold = std::max(0.06, std::min(0.12, config_.voxel_leaf_size * 2.0));
    std::vector<const pcl::PointXYZ *> plane_sample =
        use_low_height_band ? first_dense_height_band(sample, inlier_threshold) : sample;
    if (plane_sample.size() < 50) {
      return best;
    }

    std::mt19937 rng(20260702);
    std::uniform_int_distribution<std::size_t> index_dist(0, plane_sample.size() - 1);
    const double max_reference_angle_deg = config_.has_expected_floor_normal ? 18.0 : 45.0;
    const int iterations = 320;

    std::vector<double> inlier_distances;
    for (int iteration = 0; iteration < iterations; ++iteration) {
      const pcl::PointXYZ *pa = plane_sample[index_dist(rng)];
      const pcl::PointXYZ *pb = plane_sample[index_dist(rng)];
      const pcl::PointXYZ *pc = plane_sample[index_dist(rng)];
      if (pa == pb || pa == pc || pb == pc) {
        continue;
      }

      std::array<double, 3> normal;
      double d = 0.0;
      if (!plane_from_points(*pa, *pb, *pc, normal, d)) {
        continue;
      }

      const double angle_to_reference = angle_degrees_between_normals(normal, reference_normal);
      if (angle_to_reference > max_reference_angle_deg || std::abs(normal[2]) < 0.45) {
        continue;
      }

      std::size_t inliers = 0;
      double residual_sum = 0.0;
      for (const auto *point : sample) {
        const double distance = std::abs(normal[0] * point->x + normal[1] * point->y + normal[2] * point->z + d);
        if (distance <= inlier_threshold) {
          ++inliers;
          residual_sum += distance;
        }
      }
      if (inliers < 50) {
        continue;
      }

      const double inlier_ratio = static_cast<double>(inliers) / static_cast<double>(sample.size());
      const double mean_residual = residual_sum / static_cast<double>(inliers);
      const double score = inlier_ratio - 0.20 * mean_residual - 0.002 * angle_to_reference;
      if (score > best.score) {
        best.valid = true;
        best.normal = normal;
        best.d = d;
        best.inlier_ratio = inlier_ratio;
        best.angle_to_reference_deg = angle_to_reference;
        best.score = score;
      }
    }

    return best;
  }

  std::vector<const pcl::PointXYZ *> first_dense_height_band(
      const std::vector<const pcl::PointXYZ *> &points, double threshold) const {
    std::vector<const pcl::PointXYZ *> band;
    if (points.empty()) {
      return band;
    }

    double min_z = std::numeric_limits<double>::infinity();
    double max_z = -std::numeric_limits<double>::infinity();
    for (const auto *point : points) {
      min_z = std::min(min_z, static_cast<double>(point->z));
      max_z = std::max(max_z, static_cast<double>(point->z));
    }
    if (!std::isfinite(min_z) || !std::isfinite(max_z) || max_z <= min_z) {
      return band;
    }

    const double bin_size = std::max(0.04, threshold);
    const std::size_t bin_count = std::min<std::size_t>(
        240, std::max<std::size_t>(1, static_cast<std::size_t>(std::ceil((max_z - min_z) / bin_size)) + 1));
    std::vector<std::size_t> histogram(bin_count, 0);
    for (const auto *point : points) {
      const std::size_t bin = std::min<std::size_t>(
          bin_count - 1,
          static_cast<std::size_t>(std::max(0.0, std::floor((point->z - min_z) / bin_size))));
      ++histogram[bin];
    }

    const auto best_bin_it = std::max_element(histogram.begin(), histogram.end());
    const std::size_t max_bin_count = best_bin_it == histogram.end() ? 0 : *best_bin_it;
    const std::size_t min_candidate_count = std::max<std::size_t>(
        50,
        std::max<std::size_t>(
            static_cast<std::size_t>(std::ceil(static_cast<double>(points.size()) * 0.01)),
            static_cast<std::size_t>(std::ceil(static_cast<double>(max_bin_count) * 0.25))));

    std::size_t floor_bin = best_bin_it == histogram.end()
                                ? 0
                                : static_cast<std::size_t>(std::distance(histogram.begin(), best_bin_it));
    for (std::size_t bin = 0; bin < histogram.size(); ++bin) {
      if (histogram[bin] >= min_candidate_count) {
        floor_bin = bin;
        break;
      }
    }

    const double dominant_z = min_z + (static_cast<double>(floor_bin) + 0.5) * bin_size;
    const double height_band = std::max(0.18, threshold * 1.5);
    for (const auto *point : points) {
      if (std::abs(static_cast<double>(point->z) - dominant_z) <= height_band) {
        band.push_back(point);
      }
    }
    return band;
  }

  NormalFit refine_plane_from_points(const std::vector<pcl::PointXYZ> &points,
                                     const std::array<double, 3> &reference_normal) const {
    NormalFit fit;
    if (points.size() < 30) {
      return fit;
    }

    Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
    for (const auto &point : points) {
      centroid += Eigen::Vector3d(point.x, point.y, point.z);
    }
    centroid /= static_cast<double>(points.size());

    Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
    for (const auto &point : points) {
      const Eigen::Vector3d centered = Eigen::Vector3d(point.x, point.y, point.z) - centroid;
      covariance += centered * centered.transpose();
    }
    covariance /= static_cast<double>(points.size());

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
    if (solver.info() != Eigen::Success) {
      return fit;
    }

    const Eigen::Vector3d eigen_normal = solver.eigenvectors().col(0);
    std::array<double, 3> normal{eigen_normal.x(), eigen_normal.y(), eigen_normal.z()};
    if (!normalize3(normal)) {
      return fit;
    }

    fit.valid = true;
    fit.normal = normal;
    fit.d = -(normal[0] * centroid.x() + normal[1] * centroid.y() + normal[2] * centroid.z());
    fit.angle_to_reference_deg = angle_degrees_between_normals(normal, reference_normal);
    fit.inlier_ratio = static_cast<double>(points.size());
    fit.score = fit.inlier_ratio;
    return fit;
  }

  NormalFit fit_lower_envelope_floor(const Cloud::Ptr &cloud, const std::array<double, 3> &reference_normal) const {
    NormalFit best;
    if (!cloud || cloud->size() < 100) {
      return best;
    }

    std::vector<const pcl::PointXYZ *> sample;
    sample.reserve(std::min<std::size_t>(cloud->size(), 80000));
    const std::size_t stride = std::max<std::size_t>(1, cloud->size() / 80000);
    double min_x = std::numeric_limits<double>::infinity();
    double min_y = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < cloud->size(); i += stride) {
      const auto &point = cloud->points[i];
      if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
        continue;
      }
      sample.push_back(&point);
      min_x = std::min(min_x, static_cast<double>(point.x));
      min_y = std::min(min_y, static_cast<double>(point.y));
    }
    if (sample.size() < 300 || !std::isfinite(min_x) || !std::isfinite(min_y)) {
      return best;
    }

    const double cell_size = 0.35;
    std::unordered_map<std::int64_t, std::vector<const pcl::PointXYZ *>> cells;
    cells.reserve(sample.size() / 4);
    for (const auto *point : sample) {
      const auto ix = static_cast<std::int64_t>(std::floor((static_cast<double>(point->x) - min_x) / cell_size));
      const auto iy = static_cast<std::int64_t>(std::floor((static_cast<double>(point->y) - min_y) / cell_size));
      const std::int64_t key = (ix << 32) ^ (iy & 0xffffffffLL);
      cells[key].push_back(point);
    }

    std::vector<pcl::PointXYZ> envelope;
    envelope.reserve(cells.size());
    for (auto &entry : cells) {
      auto &points = entry.second;
      if (points.size() < 6) {
        continue;
      }
      std::sort(points.begin(), points.end(), [](const auto *left, const auto *right) {
        return left->z < right->z;
      });
      const std::size_t quantile_index = std::min<std::size_t>(
          points.size() - 1, static_cast<std::size_t>(std::floor(0.05 * static_cast<double>(points.size() - 1))));
      const double z_limit = static_cast<double>(points[quantile_index]->z) + 0.05;
      double sx = 0.0;
      double sy = 0.0;
      double sz = 0.0;
      std::size_t count = 0;
      for (const auto *point : points) {
        if (static_cast<double>(point->z) > z_limit) {
          break;
        }
        sx += point->x;
        sy += point->y;
        sz += point->z;
        ++count;
      }
      if (count == 0) {
        continue;
      }
      pcl::PointXYZ out;
      out.x = static_cast<float>(sx / static_cast<double>(count));
      out.y = static_cast<float>(sy / static_cast<double>(count));
      out.z = static_cast<float>(sz / static_cast<double>(count));
      envelope.push_back(out);
    }
    if (envelope.size() < 80) {
      return best;
    }

    std::mt19937 rng(20260703);
    std::uniform_int_distribution<std::size_t> index_dist(0, envelope.size() - 1);
    const double threshold = std::max(0.06, std::min(0.10, config_.voxel_leaf_size * 2.0));
    const double max_reference_angle_deg = config_.has_expected_floor_normal ? 12.0 : 45.0;
    const int iterations = 900;

    for (int iteration = 0; iteration < iterations; ++iteration) {
      const auto &pa = envelope[index_dist(rng)];
      const auto &pb = envelope[index_dist(rng)];
      const auto &pc = envelope[index_dist(rng)];

      std::array<double, 3> normal;
      double d = 0.0;
      if (!plane_from_points(pa, pb, pc, normal, d)) {
        continue;
      }
      const double angle_to_reference = angle_degrees_between_normals(normal, reference_normal);
      if (angle_to_reference > max_reference_angle_deg || std::abs(normal[2]) < 0.50) {
        continue;
      }

      std::vector<pcl::PointXYZ> inliers;
      inliers.reserve(envelope.size());
      double residual_sum = 0.0;
      for (const auto &point : envelope) {
        const double residual = std::abs(normal[0] * point.x + normal[1] * point.y + normal[2] * point.z + d);
        if (residual <= threshold) {
          inliers.push_back(point);
          residual_sum += residual;
        }
      }
      if (inliers.size() < 80) {
        continue;
      }

      NormalFit refined = refine_plane_from_points(inliers, reference_normal);
      if (!refined.valid || refined.angle_to_reference_deg > max_reference_angle_deg) {
        continue;
      }
      const double inlier_ratio = static_cast<double>(inliers.size()) / static_cast<double>(envelope.size());
      const double mean_residual = residual_sum / static_cast<double>(inliers.size());
      refined.inlier_ratio = inlier_ratio;
      refined.score = 4.0 * inlier_ratio - mean_residual - 0.01 * refined.angle_to_reference_deg;
      if (refined.score > best.score) {
        best = refined;
      }
    }
    return best;
  }

  NormalFit refine_normal_from_floor_band(const Cloud::Ptr &cloud, const std::array<double, 3> &normal,
                                          double d, const std::array<double, 3> &reference_normal) const {
    NormalFit fit;
    std::vector<Eigen::Vector3d> support;
    support.reserve(50000);
    const std::size_t stride = std::max<std::size_t>(1, cloud->size() / 50000);
    for (std::size_t i = 0; i < cloud->size(); i += stride) {
      const auto &point = cloud->points[i];
      if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
        continue;
      }
      const double height = normal[0] * point.x + normal[1] * point.y + normal[2] * point.z + d;
      if (height >= -0.04 && height <= 0.08) {
        support.emplace_back(point.x, point.y, point.z);
      }
    }
    if (support.size() < 300) {
      return fit;
    }

    Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
    for (const auto &point : support) {
      centroid += point;
    }
    centroid /= static_cast<double>(support.size());

    Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
    for (const auto &point : support) {
      const Eigen::Vector3d centered = point - centroid;
      covariance += centered * centered.transpose();
    }
    covariance /= static_cast<double>(support.size());

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
    if (solver.info() != Eigen::Success) {
      return fit;
    }

    Eigen::Vector3d eigen_normal = solver.eigenvectors().col(0);
    std::array<double, 3> refined_normal{eigen_normal.x(), eigen_normal.y(), eigen_normal.z()};
    if (!normalize3(refined_normal)) {
      return fit;
    }
    const double angle_to_reference = angle_degrees_between_normals(refined_normal, reference_normal);
    const double angle_to_initial = angle_degrees_between_normals(refined_normal, normal);
    if (angle_to_reference > 45.0 || angle_to_initial > 12.0) {
      return fit;
    }

    fit.valid = true;
    fit.normal = refined_normal;
    fit.d = -(refined_normal[0] * centroid.x() + refined_normal[1] * centroid.y() +
              refined_normal[2] * centroid.z());
    fit.inlier_ratio = static_cast<double>(support.size()) / static_cast<double>(cloud->size());
    fit.angle_to_reference_deg = angle_to_reference;
    fit.score = fit.inlier_ratio;
    return fit;
  }

  void score_candidate(const std::vector<double> &signed_distances, Candidate &candidate) const {
    size_t obstacle = 0;
    size_t below = 0;
    size_t near = 0;
    size_t floor_support = 0;
    const double below_threshold = -std::max(0.20, std::abs(config_.min_z));
    for (const double signed_distance : signed_distances) {
      const double height = signed_distance + candidate.d;
      if (height >= config_.min_z && height <= config_.max_z) {
        ++obstacle;
      }
      if (height < below_threshold) {
        ++below;
      }
      if (std::abs(height) <= 0.08) {
        ++near;
      }
      if (height >= -0.04 && height <= 0.08) {
        ++floor_support;
      }
    }

    const double total = static_cast<double>(signed_distances.size());
    candidate.obstacle_ratio = static_cast<double>(obstacle) / total;
    candidate.below_ratio = static_cast<double>(below) / total;
    candidate.near_ratio = static_cast<double>(near) / total;
    candidate.floor_support_ratio = static_cast<double>(floor_support) / total;
    if (candidate.obstacle_ratio < config_.auto_floor_min_obstacle_ratio ||
        candidate.obstacle_ratio > config_.auto_floor_max_obstacle_ratio ||
        candidate.below_ratio > config_.auto_floor_max_below_ratio) {
      candidate.score = -1.0;
      return;
    }

    const double target = std::max(1e-6, config_.auto_floor_target_obstacle_ratio);
    const double ratio_score =
        std::max(0.0, 1.0 - std::abs(candidate.obstacle_ratio - target) / std::max(target, 0.05));
    const double below_score =
        std::max(0.0, 1.0 - candidate.below_ratio / std::max(config_.auto_floor_max_below_ratio, 1e-6));
    const double near_score = std::min(1.0, candidate.near_ratio / 0.04);
    const double floor_support_score = std::min(1.0, candidate.floor_support_ratio / 0.02);
    const double early_layer_penalty =
        candidate.quantile >= 0.0 && candidate.quantile < 0.07 ? (0.07 - candidate.quantile) * 12.0 : 0.0;
    candidate.score =
        3.5 * ratio_score + 1.5 * below_score + 2.5 * floor_support_score + near_score -
        early_layer_penalty;
  }

  const Config &config_;
};

Bounds compute_bounds(const Cloud::Ptr &cloud, const Config &config) {
  Bounds bounds;
  for (const auto &point : cloud->points) {
    const auto [a, b] = projected_values(point, config);
    bounds.min_x = std::min(bounds.min_x, a);
    bounds.min_y = std::min(bounds.min_y, b);
    bounds.max_x = std::max(bounds.max_x, a);
    bounds.max_y = std::max(bounds.max_y, b);
  }
  return bounds;
}

int to_cell(double value, double origin, double resolution) {
  return static_cast<int>(std::floor((value - origin) / resolution));
}

void inflate_grid(std::vector<std::uint8_t> &grid, int width, int height, double resolution, double radius) {
  if (radius <= 0.0) {
    return;
  }
  const int cell_radius = static_cast<int>(std::ceil(radius / resolution));
  if (cell_radius <= 0) {
    return;
  }

  std::vector<std::uint8_t> inflated = grid;
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      if (grid[y * width + x] != kOccupiedValue) {
        continue;
      }
      for (int dy = -cell_radius; dy <= cell_radius; ++dy) {
        for (int dx = -cell_radius; dx <= cell_radius; ++dx) {
          if (dx * dx + dy * dy > cell_radius * cell_radius) {
            continue;
          }
          const int nx = x + dx;
          const int ny = y + dy;
          if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
            inflated[ny * width + nx] = kOccupiedValue;
          }
        }
      }
    }
  }
  grid.swap(inflated);
}

void remove_small_components(std::vector<std::uint8_t> &grid, int width, int height, int min_component_cells,
                             bool unknown_as_free, Stats &stats) {
  if (min_component_cells <= 1) {
    return;
  }

  std::vector<std::uint8_t> visited(grid.size(), 0);
  const std::uint8_t replacement = unknown_as_free ? kFreeValue : kUnknownValue;
  std::vector<int> component;
  component.reserve(static_cast<std::size_t>(min_component_cells));

  auto index = [width](int x, int y) {
    return y * width + x;
  };

  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      const int start = index(x, y);
      if (visited[start] || grid[start] != kOccupiedValue) {
        continue;
      }

      component.clear();
      std::queue<std::pair<int, int>> queue;
      queue.push({x, y});
      visited[start] = 1;

      while (!queue.empty()) {
        const auto [cx, cy] = queue.front();
        queue.pop();
        component.push_back(index(cx, cy));

        for (int dy = -1; dy <= 1; ++dy) {
          for (int dx = -1; dx <= 1; ++dx) {
            if (dx == 0 && dy == 0) {
              continue;
            }
            const int nx = cx + dx;
            const int ny = cy + dy;
            if (nx < 0 || nx >= width || ny < 0 || ny >= height) {
              continue;
            }
            const int ni = index(nx, ny);
            if (!visited[ni] && grid[ni] == kOccupiedValue) {
              visited[ni] = 1;
              queue.push({nx, ny});
            }
          }
        }
      }

      if (static_cast<int>(component.size()) < min_component_cells) {
        ++stats.removed_components;
        stats.removed_component_cells += component.size();
        for (const int cell : component) {
          grid[cell] = replacement;
        }
      }
    }
  }
}

std::vector<std::uint8_t> project_to_grid(const Cloud::Ptr &cloud, const Config &config, Stats &stats) {
  const Bounds bounds = compute_bounds(cloud, config);
  if (!bounds.valid()) {
    throw std::runtime_error("No valid points after filtering");
  }

  stats.projected_bounds = bounds;
  stats.origin_x = bounds.min_x - config.padding_cells * config.resolution;
  stats.origin_y = bounds.min_y - config.padding_cells * config.resolution;
  const double max_x = bounds.max_x + config.padding_cells * config.resolution;
  const double max_y = bounds.max_y + config.padding_cells * config.resolution;
  stats.width = std::max(1, static_cast<int>(std::ceil((max_x - stats.origin_x) / config.resolution)) + 1);
  stats.height = std::max(1, static_cast<int>(std::ceil((max_y - stats.origin_y) / config.resolution)) + 1);

  std::vector<std::uint8_t> grid(
      static_cast<std::size_t>(stats.width) * static_cast<std::size_t>(stats.height),
      config.unknown_as_free ? kFreeValue : kUnknownValue);

  for (const auto &point : cloud->points) {
    const auto [projected_x, projected_y] = projected_values(point, config);
    const int x = to_cell(projected_x, stats.origin_x, config.resolution);
    const int y = to_cell(projected_y, stats.origin_y, config.resolution);
    if (x >= 0 && x < stats.width && y >= 0 && y < stats.height) {
      grid[static_cast<std::size_t>(y) * static_cast<std::size_t>(stats.width) + static_cast<std::size_t>(x)] =
          kOccupiedValue;
    }
  }

  stats.occupied_cells_before_inflation =
      static_cast<std::size_t>(std::count(grid.begin(), grid.end(), kOccupiedValue));
  remove_small_components(grid, stats.width, stats.height, config.min_component_cells, config.unknown_as_free, stats);
  stats.occupied_cells_after_component_filter =
      static_cast<std::size_t>(std::count(grid.begin(), grid.end(), kOccupiedValue));
  inflate_grid(grid, stats.width, stats.height, config.resolution, config.inflate_radius);
  stats.occupied_cells_after_inflation =
      static_cast<std::size_t>(std::count(grid.begin(), grid.end(), kOccupiedValue));
  stats.free_cells = static_cast<std::size_t>(std::count(grid.begin(), grid.end(), kFreeValue));
  stats.unknown_cells = static_cast<std::size_t>(std::count(grid.begin(), grid.end(), kUnknownValue));
  stats.projection_plane = projection_plane_for_config(config);
  return grid;
}

void write_pgm(const std::string &path, const std::vector<std::uint8_t> &grid, int width, int height) {
  ensure_parent_directory(path);
  std::ofstream out(path, std::ios::binary);
  if (!out) {
    throw std::runtime_error("Failed to write PGM: " + path);
  }

  out << "P5\n" << width << " " << height << "\n255\n";
  for (int y = height - 1; y >= 0; --y) {
    const auto *row = grid.data() + static_cast<std::size_t>(y) * static_cast<std::size_t>(width);
    out.write(reinterpret_cast<const char *>(row), width);
  }
}

void write_yaml(const Config &config, const Stats &stats) {
  ensure_parent_directory(config.output_yaml);
  std::ofstream out(config.output_yaml);
  if (!out) {
    throw std::runtime_error("Failed to write YAML: " + config.output_yaml);
  }

  const std::filesystem::path pgm_path(config.output_pgm);
  out << "image: " << pgm_path.filename().string() << "\n";
  out << "mode: trinary\n";
  out << "resolution: " << std::fixed << std::setprecision(6) << config.resolution << "\n";
  out << "origin: [" << stats.origin_x << ", " << stats.origin_y << ", 0.000000]\n";
  out << "negate: 0\n";
  out << "occupied_thresh: " << config.occupied_threshold << "\n";
  out << "free_thresh: " << config.free_threshold << "\n";
}

std::string json_number(double value) {
  if (!std::isfinite(value)) {
    return "null";
  }
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(6) << value;
  return stream.str();
}

void write_metadata(const Config &config, const Stats &stats) {
  ensure_parent_directory(config.metadata);
  std::ofstream out(config.metadata);
  if (!out) {
    throw std::runtime_error("Failed to write metadata: " + config.metadata);
  }

  out << "{\n";
  out << "  \"input\": \"" << config.input << "\",\n";
  out << "  \"output_pgm\": \"" << config.output_pgm << "\",\n";
  out << "  \"output_yaml\": \"" << config.output_yaml << "\",\n";
  out << "  \"resolution\": " << config.resolution << ",\n";
  out << "  \"voxel_leaf_size\": " << config.voxel_leaf_size << ",\n";
  out << "  \"min_z_filter\": " << json_number(config.min_z) << ",\n";
  out << "  \"max_z_filter\": " << json_number(config.max_z) << ",\n";
  out << "  \"height_axis\": \"" << config.height_axis << "\",\n";
  out << "  \"projection_plane\": \"" << stats.projection_plane << "\",\n";
  out << "  \"floor_plane_used\": " << (config.use_floor_plane ? "true" : "false") << ",\n";
  out << "  \"floor_plane\": [" << json_number(config.floor_normal[0]) << ", "
      << json_number(config.floor_normal[1]) << ", "
      << json_number(config.floor_normal[2]) << ", "
      << json_number(config.floor_d) << "],\n";
  out << "  \"auto_floor_requested\": " << (stats.auto_floor_requested ? "true" : "false") << ",\n";
  out << "  \"auto_floor_used\": " << (stats.auto_floor_used ? "true" : "false") << ",\n";
  out << "  \"auto_floor_status\": \"" << stats.auto_floor_status << "\",\n";
  out << "  \"auto_floor_score\": " << json_number(stats.auto_floor_score) << ",\n";
  out << "  \"auto_floor_obstacle_ratio\": " << json_number(stats.auto_floor_obstacle_ratio) << ",\n";
  out << "  \"auto_floor_below_ratio\": " << json_number(stats.auto_floor_below_ratio) << ",\n";
  out << "  \"auto_floor_near_ratio\": " << json_number(stats.auto_floor_near_ratio) << ",\n";
  out << "  \"auto_floor_angle_to_expected_deg\": "
      << json_number(stats.auto_floor_angle_to_expected_deg) << ",\n";
  out << "  \"auto_floor_quantile\": " << json_number(stats.auto_floor_quantile) << ",\n";
  out << "  \"auto_floor_expected_normal\": [" << json_number(stats.auto_floor_expected_normal[0]) << ", "
      << json_number(stats.auto_floor_expected_normal[1]) << ", "
      << json_number(stats.auto_floor_expected_normal[2]) << "],\n";
  out << "  \"auto_floor_normal_fit_used\": " << (stats.auto_floor_normal_fit_used ? "true" : "false") << ",\n";
  out << "  \"auto_floor_normal_inlier_ratio\": "
      << json_number(stats.auto_floor_normal_inlier_ratio) << ",\n";
  out << "  \"min_z_seen\": " << json_number(stats.min_z_seen) << ",\n";
  out << "  \"max_z_seen\": " << json_number(stats.max_z_seen) << ",\n";
  out << "  \"unknown_as_free\": " << (config.unknown_as_free ? "true" : "false") << ",\n";
  out << "  \"radius_outlier\": " << (config.radius_outlier ? "true" : "false") << ",\n";
  out << "  \"radius_search\": " << config.radius_search << ",\n";
  out << "  \"min_neighbors\": " << config.min_neighbors << ",\n";
  out << "  \"inflate_radius\": " << config.inflate_radius << ",\n";
  out << "  \"min_component_cells\": " << config.min_component_cells << ",\n";
  out << "  \"raw_points\": " << stats.raw_points << ",\n";
  out << "  \"transformed_points\": " << stats.transformed_points << ",\n";
  out << "  \"voxel_points\": " << stats.voxel_points << ",\n";
  out << "  \"height_filtered_points\": " << stats.height_filtered_points << ",\n";
  out << "  \"radius_filtered_points\": " << stats.radius_filtered_points << ",\n";
  out << "  \"occupied_cells_before_inflation\": " << stats.occupied_cells_before_inflation << ",\n";
  out << "  \"occupied_cells_after_component_filter\": " << stats.occupied_cells_after_component_filter << ",\n";
  out << "  \"occupied_cells_after_inflation\": " << stats.occupied_cells_after_inflation << ",\n";
  out << "  \"removed_components\": " << stats.removed_components << ",\n";
  out << "  \"removed_component_cells\": " << stats.removed_component_cells << ",\n";
  out << "  \"free_cells\": " << stats.free_cells << ",\n";
  out << "  \"unknown_cells\": " << stats.unknown_cells << ",\n";
  out << "  \"width\": " << stats.width << ",\n";
  out << "  \"height\": " << stats.height << ",\n";
  out << "  \"origin\": [" << json_number(stats.origin_x) << ", " << json_number(stats.origin_y) << ", 0.000000],\n";
  out << "  \"projected_bounds_min\": [" << json_number(stats.projected_bounds.min_x) << ", "
      << json_number(stats.projected_bounds.min_y) << "],\n";
  out << "  \"projected_bounds_max\": [" << json_number(stats.projected_bounds.max_x) << ", "
      << json_number(stats.projected_bounds.max_y) << "]\n";
  out << "}\n";
}

class PointCloudMapConverter {
public:
  explicit PointCloudMapConverter(Config config) : config_(std::move(config)) {}

  Stats convert() {
    Stats stats;
    Cloud::Ptr cloud = load_pcd(config_.input);
    stats.raw_points = cloud->size();

    cloud = transform_cloud(cloud, config_);
    stats.transformed_points = cloud->size();

    cloud = voxel_downsample(cloud, config_.voxel_leaf_size);
    stats.voxel_points = cloud->size();

    FloorPlaneAnalyzer analyzer(config_);
    if (!analyzer.apply(cloud, config_, stats)) {
      throw std::runtime_error("auto floor plane analysis failed: " + stats.auto_floor_status);
    }

    cloud = filter_height_axis(cloud, config_, stats);
    stats.height_filtered_points = cloud->size();

    cloud = filter_radius_outliers(cloud, config_);
    stats.radius_filtered_points = cloud->size();

    const std::vector<std::uint8_t> grid = project_to_grid(cloud, config_, stats);
    write_pgm(config_.output_pgm, grid, stats.width, stats.height);
    write_yaml(config_, stats);
    write_metadata(config_, stats);
    return stats;
  }

  const Config &config() const {
    return config_;
  }

private:
  Config config_;
};

}  // namespace

int main(int argc, char **argv) {
  try {
    PointCloudMapConverter converter(parse_args(argc, argv));
    const Stats stats = converter.convert();
    const Config &config = converter.config();

    std::cout << "Converted PCD to PGM map\n";
    std::cout << "  input: " << config.input << "\n";
    std::cout << "  pgm: " << config.output_pgm << "\n";
    std::cout << "  yaml: " << config.output_yaml << "\n";
    std::cout << "  metadata: " << config.metadata << "\n";
    std::cout << "  raw points: " << stats.raw_points << "\n";
    std::cout << "  voxel points: " << stats.voxel_points << "\n";
    std::cout << "  height filtered points: " << stats.height_filtered_points << "\n";
    std::cout << "  final points: " << stats.radius_filtered_points << "\n";
    std::cout << "  occupied cells: " << stats.occupied_cells_after_inflation << "\n";
    std::cout << "  grid: " << stats.width << " x " << stats.height << "\n";
    std::cout << "  origin: [" << stats.origin_x << ", " << stats.origin_y << ", 0]\n";
    if (stats.auto_floor_requested) {
      std::cout << "  auto floor: " << stats.auto_floor_status
                << " used=" << (stats.auto_floor_used ? "true" : "false")
                << " score=" << stats.auto_floor_score
                << " obstacle_ratio=" << stats.auto_floor_obstacle_ratio
                << " below_ratio=" << stats.auto_floor_below_ratio
                << " quantile=" << stats.auto_floor_quantile
                << " normal_fit=" << (stats.auto_floor_normal_fit_used ? "true" : "false")
                << " normal_inliers=" << stats.auto_floor_normal_inlier_ratio << "\n";
      std::cout << "  floor plane: [" << config.floor_normal[0] << ", "
                << config.floor_normal[1] << ", " << config.floor_normal[2] << ", "
                << config.floor_d << "]\n";
    }
  } catch (const std::exception &error) {
    std::cerr << "Error: " << error.what() << "\n";
    std::cerr << "Use --help for usage.\n";
    return 1;
  }

  return 0;
}
