// Route-table data structures used to track planned motions and inter-robot dependencies.

#ifndef ROUTETABLE_H
#define ROUTETABLE_H

#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <memory>
#include <vector>
#include <algorithm>
#include <functional>
#include <stack>
#include <queue>
#include <eigen3/Eigen/Dense>
#include <fleet_manager/multi_agent_route_planner.h>

#include <fleet_manager/eses.h>

#include "fms_msgs/msg/route_table.hpp"

namespace FMS
{
    /* Node in Route Table */
    class NodeRT
    {
    public:
        NodeRT(){remove = false; init_node = false; is_goal = false;};
        int robot_id;               // Robot ID
        uint32_t id_end;            // Start node id
        uint32_t id_start;          // End node id
        Eigen::Vector3d start;      // Start pose
        Eigen::Vector3d end;        // End pose
        int timestep;               // Time step
        int state;                  // Node state: [0: staged], [1: enqueued], [2: in-progress], [3: finished], [4: dummy node], [5: occupied]
        bool remove;                // True: remove this node
        bool init_node;             // True: this is an initial node
        bool is_goal;               // True: this is a goal node
        bool is_waypoint;           // True: this is a waypoint node
        bool is_fixed = false;       // True: this is a fixed node
        bool edge_dependency = false; // True: this node has edge dependency
        std::vector<NodeRT*> preconditions;
    };
    /* Route Table Variable */
    typedef std::vector<std::vector<NodeRT*>> Table;

    class LineInfo {
    public:
        int line_start; // start timestep
        int line_end;   // end timestep
        int robot_id;        // index in the wp_route_table
        int line_id;         // node.id_start
    };

    /* Route Table Class */
    class RouteTable
    {
    /* Ref: Hnig et al. 2018: Persistent and Robust Execution of MAPF Schedules in Warehouses, IEEE ROBOTICS AND AUTOMATION LETTERS */
    public:
        /*  Class constructor  */
        RouteTable(){};
        /*  Class destructor  */
        ~RouteTable(){releaseRouteTableNodes(true);};
        /*  Clear current route table  */
        void clearRouteTable();
        /* Set parameters for route table */
        void setParams(int num_of_robots_, bool add_init_table_, int init_step_size_);
        /* Generate an initial route table by computing a commit cut in route table (ref: Hnig et al. 2018) */
        void setInitRouteTable(std::vector<bool> need_replanning, std::vector<int> loc_ids);
        /* Compute the committed steps (ref: Hnig et al. 2018) */
        void getCommittedSteps();
        /* Generate the initial route table by cutting up to the committed steps */
        void computeInitRouteTable();
        /* Clear all nodes for robots whose paths need to be replanned */
        void clearRouteTableOfSelectedRobots(std::vector<bool> need_replanning, std::vector<int> loc_ids);
        /* Remove preconditions pointing to deleted nodes (Remove the dependency relations (edges) of deleted nodes) */
        void updatePreconditions(Table& new_route_table);
        /* Get the start locations for MAPF (if we use the initial table, set the start locations to the last nodes of initial route table) */
        std::vector<std::pair<bool, Eigen::Vector3d>> getStartNodes();
        /* Set a new route table based on the computed multi-agent paths */
        void setRouteTable(Routes routes, std::vector<int> loc_ids, std::vector<Eigen::Vector3d> curr_poses);
        void find_edge_conflict(std::vector<Eigen::Vector3d> curr_poses);
        /* Append the goal station nodes in route table if it removes the station nodes */
        void setGoalStationNodes(std::vector<Eigen::Vector3d> goal_poses);
        /* Update the node states in route table */
        void updateRouteStates(std::vector<Eigen::Vector2d> poses);
        /* Get a global route of a robot from route table*/
        std::vector<Eigen::Vector3d> getGlobalRoute(int robot_i);
        /* Get a local route of a robot from route table*/
        std::vector<Eigen::Vector3d> getLocalRoute(int robot_i, std::vector<Eigen::Vector2d> curr_pose);
        /* Release (all or selected) nodes in route table */
        void releaseRouteTableNodes(bool delete_all);
        /* Switch a node with precondition nodes if switchable is true */
        void switchRouteTableNodes(std::vector<Eigen::Vector2d> poses);
        /* set preconditions in waypoints route table */
        bool set_preconditions(const Table& curr_route_talbe, std::vector<LineInfo> lines, LineInfo start, int edge_size);

        void set_result(std::string result_test_dir_);
        void set_modes(int fms_mode_, int optimizer_);

        /*Get a route table*/
        fms_msgs::msg::RouteTable getRouteTable();

        /* Get overlap waypoint lines in the wp_route_table */
        std::vector<std::vector<LineInfo>> getOverlapLines(const std::vector<std::vector<NodeRT>>& wp_route_table);
        /* Helper function that finds and returns nodes connected from the current node. */
        std::vector<std::pair<int, int>> getSuccessors(const Table& curr_route_table, int robot_id, int timestep);
        /* Check if adding a target node to route_table creates a cycle */
        bool is_cyclic(const Table& curr_route_table, int start_robot_id, int start_timestep, int target_robot_id, int target_timestep);

        bool doIntersect(Eigen::Vector2d p1, Eigen::Vector2d q1, Eigen::Vector2d p2, Eigen::Vector2d q2);

        int get_current_action(int agent);

        Table route_table;                   // Main route table
        std::vector<int> current_steps;      // Current node index in route table
        std::vector<int> committed_steps;    // Committed node index in route table
        std::vector<std::vector<LineInfo>> overlap_lines; // Overlapping lines in the wp route table

        std::vector<std::pair<LineInfo, LineInfo>> edges;


        int num_of_robots;                   // Number of robots
        int init_step_size;                  // Default step size for initial route table
        bool add_init_table;                 // True: use an initial route table for persistent planning & execution (ref: Hnig et al. 2018)

        Table copyRouteTable();

        std::string result_test_dir;
        int fms_mode;
        int optimizer;

        void runESES();

        ESES* eses;

        rclcpp::Time start_time;
    };



}

#endif
