// Declarations for the multi-agent route planning module.

#ifndef MARP_H
#define MARP_H

#include <fleet_manager/task_states.hpp>
#include <fleet_manager/robot_states.hpp>
#include <fleet_manager/topological_map.h>
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <ECBS/ecbs.h>
#include <queue>

namespace FMS
{
    /* Node in Routes */
    class NodeRoute
    {
    public:
        NodeRoute(){};
        Eigen::Vector3d pose;   // Waypoint pose [x, y, yaw]
        int loc_id;             // Location id in topological map
        bool is_dummy;          // True: dummy node
        bool is_waypoint;       // True: waypoint node
    };
    /* Routes Variable */
    typedef std::vector<std::vector<NodeRoute>> Routes;

    /* MultiAgentRoutePlanner: Path Planning Module */
    class MultiAgentRoutePlanner
    {
    public:
        /*  Class constructor  */
        MultiAgentRoutePlanner(){};
        /*  Class destructor  */
        ~MultiAgentRoutePlanner(){delete solver; delete path_planner;};
        /*  Initialize the MARP module  */
        void initMARP(TaskStates* task_states_, RobotStates* robot_states_, TopologicalMap* topolomap_);
        /*  Set the parameter of MARP module  */
        void setParams(int window_, int window_extend_, int time_limit_, double suboptimal_bound_);
        /*  Initialize the MAPF solver  */
        void initSolver();
        /*  Run MAPF solver and get the final routes (Run RHCR algorithm with ECBS solver)  */
        bool runMAPFSolver();
        /*  Initialize the solver's map from topological map  */
        void setSolverMap();
        /* Reset the solver's map from updated topological map */
        void updateSolverMap();
        /*  Set the start and goal locations from the robot state tables  */
        void setStartsAndGoals(std::vector<std::pair<bool, Eigen::Vector3d>> start_nodes);
        void setStartsAndGoals(std::vector<Eigen::Vector3d> curr_poses, std::vector<std::pair<bool, Eigen::Vector3d>> start_nodes);
        /*  Update heuristic information of the start and goal nodes for A* path planning  */
        void updateHeuristics();
        /*  Correct an overlapping start to a neighbor location for MAPF solver  */
        void correctOverlappingStarts();
        /*  Set robot stations for dummy path  */
        void setStations();
        /*  Update the routes from the computed MAPF solution  */
        void setRoutesFromSolution();

        void setModes(int fms_mode_);

        /*  Get the direction number (Right:0, Down:1, Left:2, Up:3) from yaw value  */
        int getDirection(double yaw_);
        /* Get closet direction of yaw angle */
        int getMinDir(double yaw, std::vector<double> yaws, std::vector<bool> check);
        /* Distance b/w two yaw angles */
        double getYawDist(double yaw1, double yaw2);

        /*********************************/
        /*  Task and Robot State Tables  */
        TaskStates* task_states;
        RobotStates* robot_states;
        /*********************************/
        int num_of_robots;              // Number of robots
        /* MAPF solver and system */
        ECBS* solver;                  // MAPF solver (ECBS)
        PathPlanner* path_planner;      // A* Planner for simgle agent path planing
        std::vector<Path> solution;     // Solution computed by the solver
        /* Variables for map */
        TopologicalMap* topolomap;       // Topological map
        SolverMap solver_map;           // Directed graph map for MAPF solver
        /* Computed routes (paths and ids) */
        Routes routes;                  // Final route computed by MAPF solver

        /* Starts and Goals */
        std::vector<Eigen::Vector3d> start_poses;              // Start poses (obtained from the robot state table)
        std::vector<Eigen::Vector3d> goal_poses;               // Goal poses (obtained from the robot state table)
        std::vector<std::pair<int, int>> start_loc;            // Start <node_id, direction_number> (Inputs for the solver)
        std::vector<std::pair<int, int>> goal_loc;             // Goal <node_id, direction_number> (Inputs for the solver)
        std::vector<double> yaws;                              // Yaw values: right, down, left, up
        std::vector<std::pair<int, int>> dummy_goal_loc;       // Goal location of dummy path

        /* User Parameters */
        bool remove_stations;                                   // True: remove station nodes
        double weight_move      = 1.0;                          // Constant weight for move action
        double weight_wait      = 1.0;                          // Constant weight for wait action
        int window              = 50;                           // Window: bounded time horizon
        int window_extend       = 60;                           // Extended window for single agent planner (A* Search)
        int time_limit          = 30;                           // Time limit for solver
        double suboptimal_bound = 5.0;                          // Suboptimal bound for ECBS

        int fms_mode;                                           // FMS mode 0: centralized, 1: semi-centralized, 2: De-centralized

    };
}

#endif // MARP_H
