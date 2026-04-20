// Helpers for parsing graph configuration files into runtime map structures.

#ifndef GraphRMF_H 
#define GraphRMF_H 

#include <iostream>
#include <fstream>
#include <eigen3/Eigen/Dense>
#include <vector>
#include <unordered_map>
#include <yaml-cpp/yaml.h>

/* Directly adopt the RMF map format 
(Ref: https://github.com/open-rmf/rmf_ros2/blob/f391bce7a06b6293946500326f1812ac77cf2d3c/rmf_fleet_adapter/src/rmf_fleet_adapter/agv/parse_graph.cpp) */

namespace rmf
{
  class GraphRMF
  {
  public:
    /// Default constructor
    GraphRMF(){};
    class Node
    {
      public:
      Node(){};
      std::size_t index;
      std::string map;
      std::string name;
      Eigen::Vector2d location;
      int zone;
    };
    class Lane
    {
      public:
      Lane(){};
      std::size_t index;
      std::size_t entry_node;
      std::size_t exit_node;
      bool bidirect;
    };
    std::vector<Node> nodes;
    std::vector<Lane> lanes;

    void save_graph(std::string file_name);
  };

  GraphRMF parse_graph(const std::string& filename);
}

#endif  // GraphRMF_H

