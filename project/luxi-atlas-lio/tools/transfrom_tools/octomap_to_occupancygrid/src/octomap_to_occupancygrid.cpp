#include <octomap/AbstractOcTree.h>
#include <octomap/OcTree.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

constexpr std::uint8_t kOccupiedValue = 0;
constexpr std::uint8_t kFreeValue = 254;
constexpr std::uint8_t kUnknownValue = 205;

struct Config {
  std::string input;
  std::string output_dir;
  std::string output_prefix;
  std::string output_pgm;
  std::string output_yaml;
  std::string metadata;
  double resolution = 0.0;
  double occupied_threshold = 0.65;
  double free_threshold = 0.196;
  double min_z = -std::numeric_limits<double>::infinity();
  double max_z = std::numeric_limits<double>::infinity();
  double inflate_radius = 0.0;
  int padding_cells = 1;
  bool unknown_as_free = false;
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

struct LeafProjection {
  double min_x = 0.0;
  double min_y = 0.0;
  double max_x = 0.0;
  double max_y = 0.0;
};

struct ProjectedLeafs {
  std::vector<LeafProjection> occupied;
  std::vector<LeafProjection> free;
};

struct Stats {
  double tree_resolution = 0.0;
  double grid_resolution = 0.0;
  std::size_t occupied_leafs_total = 0;
  std::size_t occupied_leafs_projected = 0;
  std::size_t free_leafs_total = 0;
  std::size_t free_leafs_projected = 0;
  std::size_t occupied_cells_before_inflation = 0;
  std::size_t occupied_cells_after_inflation = 0;
  std::size_t free_cells = 0;
  std::size_t unknown_cells = 0;
  int width = 0;
  int height = 0;
  double origin_x = 0.0;
  double origin_y = 0.0;
  double min_z_seen = std::numeric_limits<double>::infinity();
  double max_z_seen = -std::numeric_limits<double>::infinity();
  Bounds projected_bounds;
};

void print_usage(const char *program) {
  std::cout
      << "Usage:\n"
      << "  " << program << " --input map.bt|map.ot --output ./maps [options]\n\n"
      << "Options:\n"
      << "  --input PATH                 Input OctoMap .bt or .ot file\n"
      << "  --output DIR                 Output directory. Writes map.pgm, map.yaml, map.json\n"
      << "  --output-prefix PATH         Output path without extension. Default: output/<input-stem>\n"
      << "  --pgm PATH                   Output PGM path. Overrides --output-prefix for image only\n"
      << "  --yaml PATH                  Output YAML path. Overrides --output-prefix for yaml only\n"
      << "  --metadata PATH              Output JSON metadata path. Default: <output-prefix>.json\n"
      << "  --resolution M               OccupancyGrid resolution. Default: OctoMap resolution\n"
      << "  --occupied-threshold P       YAML occupied threshold, default 0.65\n"
      << "  --free-threshold P           YAML free threshold, default 0.196\n"
      << "  --min-z M                    Minimum voxel center height to project\n"
      << "  --max-z M                    Maximum voxel center height to project\n"
      << "  --inflate-radius M           Inflate occupied cells by this radius in meters. Default: 0\n"
      << "  --padding-cells N            Padding around projected map. Default: 1\n"
      << "  --unknown-as-free            Write unknown cells as free\n"
      << "  --help                       Show this help\n\n"
      << "Examples:\n"
      << "  " << program << " --input map.bt --resolution 0.05 --output ./maps\n"
      << "  " << program << " --input map.bt --min-z -0.3 --max-z 2.0 --unknown-as-free --output ./maps_nav\n";
}

bool has_suffix(const std::string &text, const std::string &suffix) {
  return text.size() >= suffix.size() && text.compare(text.size() - suffix.size(), suffix.size(), suffix) == 0;
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
      if (config.resolution <= 0.0) {
        throw std::runtime_error("--resolution must be > 0");
      }
    } else if (arg == "--occupied-threshold") {
      config.occupied_threshold = parse_double(need_value(arg), arg);
    } else if (arg == "--free-threshold") {
      config.free_threshold = parse_double(need_value(arg), arg);
    } else if (arg == "--min-z") {
      config.min_z = parse_double(need_value(arg), arg);
    } else if (arg == "--max-z") {
      config.max_z = parse_double(need_value(arg), arg);
    } else if (arg == "--inflate-radius") {
      config.inflate_radius = parse_double(need_value(arg), arg);
    } else if (arg == "--padding-cells") {
      config.padding_cells = parse_int(need_value(arg), arg);
    } else if (arg == "--unknown-as-free") {
      config.unknown_as_free = true;
    } else {
      throw std::runtime_error("Unknown argument: " + arg);
    }
  }

  if (config.input.empty()) {
    throw std::runtime_error("--input is required");
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

  if (config.occupied_threshold <= 0.0 || config.occupied_threshold >= 1.0) {
    throw std::runtime_error("--occupied-threshold must be between 0 and 1");
  }
  if (config.free_threshold <= 0.0 || config.free_threshold >= 1.0) {
    throw std::runtime_error("--free-threshold must be between 0 and 1");
  }
  if (config.free_threshold >= config.occupied_threshold) {
    throw std::runtime_error("--free-threshold must be less than --occupied-threshold");
  }
  if (config.min_z > config.max_z) {
    throw std::runtime_error("--min-z must be <= --max-z");
  }
  if (config.inflate_radius < 0.0) {
    throw std::runtime_error("--inflate-radius must be >= 0");
  }
  if (config.padding_cells < 0) {
    throw std::runtime_error("--padding-cells must be >= 0");
  }

  return config;
}

std::unique_ptr<octomap::OcTree> load_octree(const std::string &path) {
  if (has_suffix(path, ".bt")) {
    std::unique_ptr<octomap::OcTree> tree(new octomap::OcTree(0.1));
    if (!tree->readBinary(path)) {
      throw std::runtime_error("Failed to read binary OctoMap: " + path);
    }
    return tree;
  }

  std::unique_ptr<octomap::AbstractOcTree> abstract_tree(octomap::AbstractOcTree::read(path));
  if (!abstract_tree) {
    throw std::runtime_error("Failed to read OctoMap: " + path);
  }

  octomap::OcTree *raw_tree = dynamic_cast<octomap::OcTree *>(abstract_tree.release());
  if (!raw_tree) {
    throw std::runtime_error("Unsupported tree type. This tool currently supports OcTree .bt/.ot files");
  }
  return std::unique_ptr<octomap::OcTree>(raw_tree);
}

void ensure_parent_directory(const std::string &path) {
  const std::filesystem::path file_path(path);
  const std::filesystem::path parent = file_path.parent_path();
  if (!parent.empty()) {
    std::filesystem::create_directories(parent);
  }
}

void expand_bounds(Bounds &bounds, const LeafProjection &leaf) {
  bounds.min_x = std::min(bounds.min_x, leaf.min_x);
  bounds.min_y = std::min(bounds.min_y, leaf.min_y);
  bounds.max_x = std::max(bounds.max_x, leaf.max_x);
  bounds.max_y = std::max(bounds.max_y, leaf.max_y);
}

ProjectedLeafs collect_projected_leafs(const octomap::OcTree &tree, const Config &config, Stats &stats) {
  ProjectedLeafs leafs;

  for (auto it = tree.begin_leafs(), end = tree.end_leafs(); it != end; ++it) {
    const bool occupied = tree.isNodeOccupied(*it);
    const double z = it.getZ();
    stats.min_z_seen = std::min(stats.min_z_seen, z);
    stats.max_z_seen = std::max(stats.max_z_seen, z);

    if (occupied) {
      ++stats.occupied_leafs_total;
    } else {
      ++stats.free_leafs_total;
    }

    if (z < config.min_z || z > config.max_z) {
      continue;
    }

    const double half = it.getSize() * 0.5;
    LeafProjection leaf;
    leaf.min_x = it.getX() - half;
    leaf.max_x = it.getX() + half;
    leaf.min_y = it.getY() - half;
    leaf.max_y = it.getY() + half;

    if (occupied) {
      leafs.occupied.push_back(leaf);
    } else {
      leafs.free.push_back(leaf);
    }
    expand_bounds(stats.projected_bounds, leaf);
  }

  stats.occupied_leafs_projected = leafs.occupied.size();
  stats.free_leafs_projected = leafs.free.size();
  return leafs;
}

int clamp_index(int value, int low, int high) {
  return std::max(low, std::min(value, high));
}

void mark_leaf_cells(
    const std::vector<LeafProjection> &leafs,
    const Stats &stats,
    std::vector<std::uint8_t> &grid,
    std::uint8_t value,
    bool only_unknown) {
  for (const auto &leaf : leafs) {
    const int min_ix = clamp_index(static_cast<int>(std::floor((leaf.min_x - stats.origin_x) / stats.grid_resolution)), 0, stats.width - 1);
    const int max_ix = clamp_index(static_cast<int>(std::floor((leaf.max_x - stats.origin_x) / stats.grid_resolution)), 0, stats.width - 1);
    const int min_iy = clamp_index(static_cast<int>(std::floor((leaf.min_y - stats.origin_y) / stats.grid_resolution)), 0, stats.height - 1);
    const int max_iy = clamp_index(static_cast<int>(std::floor((leaf.max_y - stats.origin_y) / stats.grid_resolution)), 0, stats.height - 1);

    for (int iy = min_iy; iy <= max_iy; ++iy) {
      for (int ix = min_ix; ix <= max_ix; ++ix) {
        const std::size_t index = static_cast<std::size_t>(iy) * static_cast<std::size_t>(stats.width) + static_cast<std::size_t>(ix);
        if (!only_unknown || grid[index] == kUnknownValue) {
          grid[index] = value;
        }
      }
    }
  }
}

void inflate_occupied_cells(const Config &config, const Stats &stats, std::vector<std::uint8_t> &grid) {
  if (config.inflate_radius <= 0.0) {
    return;
  }

  const int inflate_cells = static_cast<int>(std::ceil(config.inflate_radius / stats.grid_resolution));
  const double radius_sq = config.inflate_radius * config.inflate_radius;
  std::vector<std::uint8_t> inflated = grid;

  for (int iy = 0; iy < stats.height; ++iy) {
    for (int ix = 0; ix < stats.width; ++ix) {
      const std::size_t index = static_cast<std::size_t>(iy) * static_cast<std::size_t>(stats.width) + static_cast<std::size_t>(ix);
      if (grid[index] != kOccupiedValue) {
        continue;
      }

      for (int dy = -inflate_cells; dy <= inflate_cells; ++dy) {
        for (int dx = -inflate_cells; dx <= inflate_cells; ++dx) {
          const double dist_sq = std::pow(dx * stats.grid_resolution, 2) + std::pow(dy * stats.grid_resolution, 2);
          if (dist_sq > radius_sq) {
            continue;
          }
          const int nx = ix + dx;
          const int ny = iy + dy;
          if (nx < 0 || ny < 0 || nx >= stats.width || ny >= stats.height) {
            continue;
          }
          const std::size_t inflated_index =
              static_cast<std::size_t>(ny) * static_cast<std::size_t>(stats.width) + static_cast<std::size_t>(nx);
          inflated[inflated_index] = kOccupiedValue;
        }
      }
    }
  }

  grid.swap(inflated);
}

void count_cells(const std::vector<std::uint8_t> &grid, Stats &stats) {
  stats.free_cells = 0;
  stats.unknown_cells = 0;
  stats.occupied_cells_after_inflation = 0;

  for (const auto cell : grid) {
    if (cell == kOccupiedValue) {
      ++stats.occupied_cells_after_inflation;
    } else if (cell == kFreeValue) {
      ++stats.free_cells;
    } else {
      ++stats.unknown_cells;
    }
  }
}

void write_pgm(const Config &config, const Stats &stats, const std::vector<std::uint8_t> &grid) {
  ensure_parent_directory(config.output_pgm);
  std::ofstream out(config.output_pgm, std::ios::binary);
  if (!out) {
    throw std::runtime_error("Failed to write PGM: " + config.output_pgm);
  }

  out << "P5\n";
  out << "# CREATOR: octomap2grid\n";
  out << stats.width << " " << stats.height << "\n";
  out << "255\n";

  for (int row = stats.height - 1; row >= 0; --row) {
    const std::size_t offset = static_cast<std::size_t>(row) * static_cast<std::size_t>(stats.width);
    out.write(reinterpret_cast<const char *>(grid.data() + offset), stats.width);
  }
}

void write_yaml(const Config &config, const Stats &stats) {
  ensure_parent_directory(config.output_yaml);
  std::ofstream out(config.output_yaml);
  if (!out) {
    throw std::runtime_error("Failed to write YAML: " + config.output_yaml);
  }

  const std::string image_name = std::filesystem::path(config.output_pgm).filename().string();
  out << "image: " << image_name << "\n";
  out << "resolution: " << std::fixed << std::setprecision(6) << stats.grid_resolution << "\n";
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
  out << "  \"tree_resolution\": " << stats.tree_resolution << ",\n";
  out << "  \"grid_resolution\": " << stats.grid_resolution << ",\n";
  out << "  \"occupied_threshold\": " << config.occupied_threshold << ",\n";
  out << "  \"free_threshold\": " << config.free_threshold << ",\n";
  out << "  \"width\": " << stats.width << ",\n";
  out << "  \"height\": " << stats.height << ",\n";
  out << "  \"origin\": [" << json_number(stats.origin_x) << ", " << json_number(stats.origin_y) << ", 0.000000],\n";
  out << "  \"min_z_filter\": " << json_number(config.min_z) << ",\n";
  out << "  \"max_z_filter\": " << json_number(config.max_z) << ",\n";
  out << "  \"min_z_seen\": " << json_number(stats.min_z_seen) << ",\n";
  out << "  \"max_z_seen\": " << json_number(stats.max_z_seen) << ",\n";
  out << "  \"inflate_radius\": " << config.inflate_radius << ",\n";
  out << "  \"unknown_as_free\": " << (config.unknown_as_free ? "true" : "false") << ",\n";
  out << "  \"occupied_leafs_total\": " << stats.occupied_leafs_total << ",\n";
  out << "  \"occupied_leafs_projected\": " << stats.occupied_leafs_projected << ",\n";
  out << "  \"free_leafs_total\": " << stats.free_leafs_total << ",\n";
  out << "  \"free_leafs_projected\": " << stats.free_leafs_projected << ",\n";
  out << "  \"occupied_cells_before_inflation\": " << stats.occupied_cells_before_inflation << ",\n";
  out << "  \"occupied_cells_after_inflation\": " << stats.occupied_cells_after_inflation << ",\n";
  out << "  \"free_cells\": " << stats.free_cells << ",\n";
  out << "  \"unknown_cells\": " << stats.unknown_cells << ",\n";
  out << "  \"projected_bounds_min\": [" << json_number(stats.projected_bounds.min_x) << ", " << json_number(stats.projected_bounds.min_y) << "],\n";
  out << "  \"projected_bounds_max\": [" << json_number(stats.projected_bounds.max_x) << ", " << json_number(stats.projected_bounds.max_y) << "]\n";
  out << "}\n";
}

}  // namespace

int main(int argc, char **argv) {
  try {
    const Config config = parse_args(argc, argv);
    const std::unique_ptr<octomap::OcTree> tree = load_octree(config.input);

    Stats stats;
    stats.tree_resolution = tree->getResolution();
    stats.grid_resolution = config.resolution > 0.0 ? config.resolution : stats.tree_resolution;

    const ProjectedLeafs leafs = collect_projected_leafs(*tree, config, stats);
    if ((leafs.occupied.empty() && leafs.free.empty()) || !stats.projected_bounds.valid()) {
      throw std::runtime_error("No OctoMap leaf voxels remained after z filtering");
    }

    stats.origin_x = std::floor(stats.projected_bounds.min_x / stats.grid_resolution) * stats.grid_resolution -
                     config.padding_cells * stats.grid_resolution;
    stats.origin_y = std::floor(stats.projected_bounds.min_y / stats.grid_resolution) * stats.grid_resolution -
                     config.padding_cells * stats.grid_resolution;

    const double max_x = std::ceil(stats.projected_bounds.max_x / stats.grid_resolution) * stats.grid_resolution +
                         config.padding_cells * stats.grid_resolution;
    const double max_y = std::ceil(stats.projected_bounds.max_y / stats.grid_resolution) * stats.grid_resolution +
                         config.padding_cells * stats.grid_resolution;

    stats.width = static_cast<int>(std::ceil((max_x - stats.origin_x) / stats.grid_resolution));
    stats.height = static_cast<int>(std::ceil((max_y - stats.origin_y) / stats.grid_resolution));
    if (stats.width <= 0 || stats.height <= 0) {
      throw std::runtime_error("Projected grid has invalid dimensions");
    }

    const std::uint8_t default_value = config.unknown_as_free ? kFreeValue : kUnknownValue;
    std::vector<std::uint8_t> grid(static_cast<std::size_t>(stats.width) * static_cast<std::size_t>(stats.height), default_value);

    if (!config.unknown_as_free) {
      mark_leaf_cells(leafs.free, stats, grid, kFreeValue, true);
    }
    mark_leaf_cells(leafs.occupied, stats, grid, kOccupiedValue, false);
    count_cells(grid, stats);
    stats.occupied_cells_before_inflation = stats.occupied_cells_after_inflation;

    inflate_occupied_cells(config, stats, grid);
    count_cells(grid, stats);

    write_pgm(config, stats, grid);
    write_yaml(config, stats);
    write_metadata(config, stats);

    std::cout << "Converted OctoMap to OccupancyGrid\n";
    std::cout << "  input: " << config.input << "\n";
    std::cout << "  pgm: " << config.output_pgm << "\n";
    std::cout << "  yaml: " << config.output_yaml << "\n";
    std::cout << "  metadata: " << config.metadata << "\n";
    std::cout << "  tree resolution: " << stats.tree_resolution << "\n";
    std::cout << "  grid resolution: " << stats.grid_resolution << "\n";
    std::cout << "  size: " << stats.width << " x " << stats.height << "\n";
    std::cout << "  origin: [" << stats.origin_x << ", " << stats.origin_y << ", 0]\n";
    std::cout << "  occupied leafs projected: " << stats.occupied_leafs_projected << " / " << stats.occupied_leafs_total << "\n";
    std::cout << "  free leafs projected: " << stats.free_leafs_projected << " / " << stats.free_leafs_total << "\n";
    std::cout << "  occupied cells: " << stats.occupied_cells_after_inflation << "\n";
    std::cout << "  free cells: " << stats.free_cells << "\n";
    std::cout << "  unknown cells: " << stats.unknown_cells << "\n";
  } catch (const std::exception &error) {
    std::cerr << "octomap2grid: " << error.what() << "\n";
    return 1;
  }

  return 0;
}
