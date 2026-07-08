#include <octomap/AbstractOcTree.h>
#include <octomap/OcTree.h>

#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>

namespace {

void print_usage(const char *program) {
  std::cout << "Usage:\n  " << program << " map.bt|map.ot\n";
}

}  // namespace

int main(int argc, char **argv) {
  if (argc != 2 || std::string(argv[1]) == "--help" || std::string(argv[1]) == "-h") {
    print_usage(argv[0]);
    return argc == 2 ? 0 : 1;
  }

  try {
    const std::string path = argv[1];
    std::unique_ptr<octomap::AbstractOcTree> abstract_tree;
    std::unique_ptr<octomap::OcTree> binary_tree;
    octomap::OcTree *tree = nullptr;

    if (path.size() >= 3 && path.substr(path.size() - 3) == ".bt") {
      binary_tree.reset(new octomap::OcTree(0.1));
      if (!binary_tree->readBinary(path)) {
        throw std::runtime_error("failed to read binary tree " + path);
      }
      tree = binary_tree.get();
    } else {
      abstract_tree.reset(octomap::AbstractOcTree::read(path));
      if (!abstract_tree) {
        throw std::runtime_error("failed to read " + path);
      }

      tree = dynamic_cast<octomap::OcTree *>(abstract_tree.get());
      if (!tree) {
        std::cout << "Loaded tree type: " << abstract_tree->getTreeType() << "\n";
        std::cout << "This inspector currently prints detailed stats for OcTree only.\n";
        return 0;
      }
    }

    std::size_t occupied_leafs = 0;
    for (auto it = tree->begin_leafs(), end = tree->end_leafs(); it != end; ++it) {
      if (tree->isNodeOccupied(*it)) {
        ++occupied_leafs;
      }
    }

    double min_x = 0.0, min_y = 0.0, min_z = 0.0;
    double max_x = 0.0, max_y = 0.0, max_z = 0.0;
    tree->getMetricMin(min_x, min_y, min_z);
    tree->getMetricMax(max_x, max_y, max_z);

    std::cout << "OctoMap file: " << argv[1] << "\n";
    std::cout << "  tree type: " << tree->getTreeType() << "\n";
    std::cout << "  resolution: " << tree->getResolution() << "\n";
    std::cout << "  tree depth: " << tree->getTreeDepth() << "\n";
    std::cout << "  total nodes: " << tree->size() << "\n";
    std::cout << "  leaf nodes: " << tree->getNumLeafNodes() << "\n";
    std::cout << "  occupied leaf nodes: " << occupied_leafs << "\n";
    std::cout << "  metric min: [" << min_x << ", " << min_y << ", " << min_z << "]\n";
    std::cout << "  metric max: [" << max_x << ", " << max_y << ", " << max_z << "]\n";
  } catch (const std::exception &error) {
    std::cerr << "inspect_octomap: " << error.what() << "\n";
    return 1;
  }

  return 0;
}
