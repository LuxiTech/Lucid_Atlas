#include <octomap/OcTree.h>
#include <octomap/Pointcloud.h>

#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

struct Config {
  std::string input;
  std::string output;
  std::string metadata;
  double resolution = 0.05;
  double voxel_leaf_size = 0.0;
  double min_z = -std::numeric_limits<double>::infinity();
  double max_z = std::numeric_limits<double>::infinity();
  double hit_prob = 0.7;
  double miss_prob = 0.4;
  double clamping_min = 0.12;
  double clamping_max = 0.97;
  double occupancy_threshold = 0.5;
  double max_range = -1.0;
  bool write_full = false;
  bool use_sensor_origin = false;
  octomap::point3d sensor_origin{0.0f, 0.0f, 0.0f};
};

struct Stats {
  std::size_t raw_points = 0;
  std::size_t downsampled_points = 0;
  std::size_t accepted_points = 0;
  std::size_t occupied_leafs = 0;
  std::size_t total_leafs = 0;
  double min_x = std::numeric_limits<double>::infinity();
  double min_y = std::numeric_limits<double>::infinity();
  double min_z = std::numeric_limits<double>::infinity();
  double max_x = -std::numeric_limits<double>::infinity();
  double max_y = -std::numeric_limits<double>::infinity();
  double max_z = -std::numeric_limits<double>::infinity();
};

void print_usage(const char *program) {
  std::cout
      << "Usage:\n"
      << "  " << program << " --input map.pcd --output map.bt [options]\n\n"
      << "Options:\n"
      << "  --input PATH                 Input PCD file\n"
      << "  --output PATH                Output .bt or .ot file\n"
      << "  --resolution M               OctoMap resolution in meters, default 0.05\n"
      << "  --voxel-leaf-size M          Optional PCD downsample size. Default: resolution\n"
      << "  --min-z M                    Minimum accepted z height\n"
      << "  --max-z M                    Maximum accepted z height\n"
      << "  --hit-prob P                 Hit probability, default 0.7\n"
      << "  --miss-prob P                Miss probability, default 0.4\n"
      << "  --clamping-min P             Min occupancy clamp, default 0.12\n"
      << "  --clamping-max P             Max occupancy clamp, default 0.97\n"
      << "  --occupancy-threshold P      Occupancy threshold, default 0.5\n"
      << "  --sensor-origin X Y Z        Insert point cloud with ray casting from this origin\n"
      << "  --max-range M                Max ray length when --sensor-origin is used\n"
      << "  --write-full                 Write full .ot tree instead of binary .bt\n"
      << "  --metadata PATH              JSON metadata output path. Default: <output>.json\n"
      << "  --help                       Show this help\n\n"
      << "Examples:\n"
      << "  " << program << " --input room_001.pcd --resolution 0.05 --output room_001.bt\n"
      << "  " << program << " --input room_001.pcd --resolution 0.10 --min-z -1 --max-z 3 --output room_001.ot --write-full\n";
}

double parse_double(const std::string &value, const std::string &name) {
  char *end = nullptr;
  const double parsed = std::strtod(value.c_str(), &end);
  if (end == value.c_str() || *end != '\0') {
    throw std::runtime_error("Invalid value for " + name + ": " + value);
  }
  return parsed;
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
      config.output = need_value(arg);
    } else if (arg == "--resolution") {
      config.resolution = parse_double(need_value(arg), arg);
    } else if (arg == "--voxel-leaf-size") {
      config.voxel_leaf_size = parse_double(need_value(arg), arg);
    } else if (arg == "--min-z") {
      config.min_z = parse_double(need_value(arg), arg);
    } else if (arg == "--max-z") {
      config.max_z = parse_double(need_value(arg), arg);
    } else if (arg == "--hit-prob") {
      config.hit_prob = parse_double(need_value(arg), arg);
    } else if (arg == "--miss-prob") {
      config.miss_prob = parse_double(need_value(arg), arg);
    } else if (arg == "--clamping-min") {
      config.clamping_min = parse_double(need_value(arg), arg);
    } else if (arg == "--clamping-max") {
      config.clamping_max = parse_double(need_value(arg), arg);
    } else if (arg == "--occupancy-threshold") {
      config.occupancy_threshold = parse_double(need_value(arg), arg);
    } else if (arg == "--max-range") {
      config.max_range = parse_double(need_value(arg), arg);
    } else if (arg == "--metadata") {
      config.metadata = need_value(arg);
    } else if (arg == "--write-full") {
      config.write_full = true;
    } else if (arg == "--sensor-origin") {
      if (i + 3 >= argc) {
        throw std::runtime_error("Missing values for --sensor-origin X Y Z");
      }
      const float x = static_cast<float>(parse_double(argv[++i], "--sensor-origin X"));
      const float y = static_cast<float>(parse_double(argv[++i], "--sensor-origin Y"));
      const float z = static_cast<float>(parse_double(argv[++i], "--sensor-origin Z"));
      config.sensor_origin = octomap::point3d(x, y, z);
      config.use_sensor_origin = true;
    } else {
      throw std::runtime_error("Unknown argument: " + arg);
    }
  }

  if (config.input.empty()) {
    throw std::runtime_error("--input is required");
  }
  if (config.output.empty()) {
    throw std::runtime_error("--output is required");
  }
  if (config.metadata.empty()) {
    config.metadata = config.output + ".json";
  }
  if (config.resolution <= 0.0) {
    throw std::runtime_error("--resolution must be > 0");
  }
  if (config.voxel_leaf_size <= 0.0) {
    config.voxel_leaf_size = config.resolution;
  }
  if (config.min_z > config.max_z) {
    throw std::runtime_error("--min-z must be <= --max-z");
  }

  return config;
}

void update_bounds(Stats &stats, const pcl::PointXYZ &point) {
  stats.min_x = std::min(stats.min_x, static_cast<double>(point.x));
  stats.min_y = std::min(stats.min_y, static_cast<double>(point.y));
  stats.min_z = std::min(stats.min_z, static_cast<double>(point.z));
  stats.max_x = std::max(stats.max_x, static_cast<double>(point.x));
  stats.max_y = std::max(stats.max_y, static_cast<double>(point.y));
  stats.max_z = std::max(stats.max_z, static_cast<double>(point.z));
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
  std::ofstream out(config.metadata);
  if (!out) {
    throw std::runtime_error("Failed to write metadata: " + config.metadata);
  }

  out << "{\n";
  out << "  \"input\": \"" << config.input << "\",\n";
  out << "  \"output\": \"" << config.output << "\",\n";
  out << "  \"format\": \"" << (config.write_full ? "ot" : "bt") << "\",\n";
  out << "  \"resolution\": " << config.resolution << ",\n";
  out << "  \"voxel_leaf_size\": " << config.voxel_leaf_size << ",\n";
  out << "  \"min_z_filter\": " << json_number(config.min_z) << ",\n";
  out << "  \"max_z_filter\": " << json_number(config.max_z) << ",\n";
  out << "  \"hit_prob\": " << config.hit_prob << ",\n";
  out << "  \"miss_prob\": " << config.miss_prob << ",\n";
  out << "  \"sensor_origin_used\": " << (config.use_sensor_origin ? "true" : "false") << ",\n";
  out << "  \"raw_points\": " << stats.raw_points << ",\n";
  out << "  \"downsampled_points\": " << stats.downsampled_points << ",\n";
  out << "  \"accepted_points\": " << stats.accepted_points << ",\n";
  out << "  \"total_leafs\": " << stats.total_leafs << ",\n";
  out << "  \"occupied_leafs\": " << stats.occupied_leafs << ",\n";
  out << "  \"bbox_min\": [" << json_number(stats.min_x) << ", " << json_number(stats.min_y) << ", " << json_number(stats.min_z) << "],\n";
  out << "  \"bbox_max\": [" << json_number(stats.max_x) << ", " << json_number(stats.max_y) << ", " << json_number(stats.max_z) << "]\n";
  out << "}\n";
}

}  // namespace

int main(int argc, char **argv) {
  try {
    const Config config = parse_args(argc, argv);

    pcl::PCLPointCloud2 raw_blob;
    if (pcl::io::loadPCDFile(config.input, raw_blob) != 0) {
      throw std::runtime_error("Failed to read PCD: " + config.input);
    }

    Stats stats;
    stats.raw_points = static_cast<std::size_t>(raw_blob.width) * static_cast<std::size_t>(raw_blob.height);

    pcl::PCLPointCloud2 filtered_blob;
    pcl::PCLPointCloud2::Ptr raw_blob_ptr(new pcl::PCLPointCloud2(raw_blob));
    pcl::VoxelGrid<pcl::PCLPointCloud2> voxel;
    voxel.setInputCloud(raw_blob_ptr);
    voxel.setLeafSize(
        static_cast<float>(config.voxel_leaf_size),
        static_cast<float>(config.voxel_leaf_size),
        static_cast<float>(config.voxel_leaf_size));
    voxel.filter(filtered_blob);

    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromPCLPointCloud2(filtered_blob, cloud);
    stats.downsampled_points = cloud.size();

    octomap::OcTree tree(config.resolution);
    tree.setProbHit(config.hit_prob);
    tree.setProbMiss(config.miss_prob);
    tree.setClampingThresMin(config.clamping_min);
    tree.setClampingThresMax(config.clamping_max);
    tree.setOccupancyThres(config.occupancy_threshold);

    if (config.use_sensor_origin) {
      octomap::Pointcloud octo_cloud;
      for (const auto &point : cloud.points) {
        if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
          continue;
        }
        if (point.z < config.min_z || point.z > config.max_z) {
          continue;
        }
        octo_cloud.push_back(point.x, point.y, point.z);
        update_bounds(stats, point);
        ++stats.accepted_points;
      }
      tree.insertPointCloud(octo_cloud, config.sensor_origin, config.max_range);
    } else {
      for (const auto &point : cloud.points) {
        if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
          continue;
        }
        if (point.z < config.min_z || point.z > config.max_z) {
          continue;
        }
        tree.updateNode(octomap::point3d(point.x, point.y, point.z), true);
        update_bounds(stats, point);
        ++stats.accepted_points;
      }
    }

    tree.updateInnerOccupancy();
    stats.total_leafs = tree.getNumLeafNodes();
    for (auto it = tree.begin_leafs(), end = tree.end_leafs(); it != end; ++it) {
      if (tree.isNodeOccupied(*it)) {
        ++stats.occupied_leafs;
      }
    }

    const bool write_ok = config.write_full ? tree.write(config.output) : tree.writeBinary(config.output);
    if (!write_ok) {
      throw std::runtime_error("Failed to write OctoMap: " + config.output);
    }

    write_metadata(config, stats);

    std::cout << "Converted PCD to OctoMap\n";
    std::cout << "  input: " << config.input << "\n";
    std::cout << "  output: " << config.output << "\n";
    std::cout << "  metadata: " << config.metadata << "\n";
    std::cout << "  raw points: " << stats.raw_points << "\n";
    std::cout << "  downsampled points: " << stats.downsampled_points << "\n";
    std::cout << "  accepted points: " << stats.accepted_points << "\n";
    std::cout << "  occupied leafs: " << stats.occupied_leafs << "\n";
    std::cout << "  total leafs: " << stats.total_leafs << "\n";
  } catch (const std::exception &error) {
    std::cerr << "pcd2octomap: " << error.what() << "\n";
    std::cerr << "Run with --help for usage.\n";
    return 1;
  }

  return 0;
}
