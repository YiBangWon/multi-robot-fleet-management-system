// Topological map definitions used across planning, control, and visualization.

#ifndef TOPLO_MAP_H
#define TOPLO_MAP_H

#include <sstream>
#include <fstream>
#include <fleet_manager/parse_graph.hpp>
#include <yaml-cpp/yaml.h>
#include <filesystem>

namespace FMS
{

class TopologicalMap
{
    public:
        struct Node{
            Eigen::Vector2d loc;        // XY-positions
            std::string name = "";      // Node name
            bool is_waypoint = false;
            bool disabled = false;
            int zone = -1;
        };
        struct Lane{
            // Indexes of entry node and exit node
            int entry;      // Entry node id
            int exit;       // Exit node id
            bool bidirect;
            bool disabled = false;
        };

        TopologicalMap(){};
        ~TopologicalMap(){};

        bool loadTopologicalMapFile(std::string filename_graph, bool is_directed_graph_,  // Load topological Map file
                                    bool remove_stations_, double unit_dist_);
        bool extendTopologicalMap(std::string filename_graph);                            // Generate the extended nodes and edges
        int getNodeId(Eigen::Vector3d pose);                                              // Get the node id from pose location
        int getNodeId(Eigen::Vector2d pose);                                              // Get the node id from pose location
        int getRouteNodeId(int robot_id,                                                  // Get the nearest node id in global path from pose location
                           std::vector<std::vector<Eigen::Vector3d>> global_path,
                           Eigen::Vector2d loc);
        bool isEqual(Eigen::Vector2d loc1, Eigen::Vector2d loc2);                         // Check two locations are equal
        int getStationId(std::string node_name);                                          // Get the node id from node name
        void setNodeName();                                                               // Set the node name
        void setNodeStation();                                                            // Set the station node for each robot
        void removeNodes(std::vector<int> station_nodes);                                 // Remove station nodes
        void disableRegion(Eigen::Vector2d bbx_min, Eigen::Vector2d bbx_max);             // Disable nodes in bounding box region
        void enableAllNodes();                                                            // Enable all nodes
        void assignDuckAndChargingStation(std::vector<Eigen::Vector3d> poses);            // Assign duck and charging station
        void saveExtendedTopologicalMap(std::string file_name_nodes, std::string file_name_lanes);  // Save the extended topolgical map
        void loadExtendedTopologicalMap(std::string file_name_nodes, std::string file_name_lanes);  // Load the extended topolgical map
        void setZone();

        rmf::GraphRMF graph;                      // Loaded RMF Graph
        int num_nodes;                            // number of nodes
        int num_lanes;                            // number of lanes
        std::vector<Node> nodes;                  // Graph nodes (xy-positions)
        std::vector<Lane> lanes;                  // Indexes of entry node and exit node
        std::vector<int> nodes_dock;              // Dock node for each robot (This is required for reserving dummy paths [ref: Liu et al. 2019])
        std::vector<int> charging_station_index;  // Node index of charing station
        std::vector<std::pair<std::string, Eigen::Vector2d>> stations; // Station nodes <name, xy-position>

        bool is_directed_graph;                   // Graph is directed or bidirected
        bool remove_stations = true;              // Remove station nodes
        double unit_dist;                         // Divide the edges on the topological map according to this unit distance (meter)
};

}

#endif // TOPLO_MAP_H
