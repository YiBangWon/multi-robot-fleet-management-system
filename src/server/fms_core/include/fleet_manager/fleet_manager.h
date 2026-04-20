// Public interface for the core fleet manager node and its shared ROS interfaces.

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "std_msgs/msg/int32_multi_array.hpp"

// ROS
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "fms_msgs/msg/tasks.hpp"
#include "fms_msgs/msg/node.hpp"
#include "fms_msgs/msg/task_info.hpp"
#include "fms_msgs/msg/robots.hpp"
#include "fms_msgs/msg/robot_state.hpp"
#include "fms_msgs/msg/robot_info.hpp"
#include "fms_msgs/msg/robot_model.hpp"

#include "fms_msgs/msg/node_rt.hpp"
#include "fms_msgs/msg/route_table_node.hpp"
#include "fms_msgs/msg/agent_route_table.hpp"
#include "fms_msgs/msg/route_table.hpp"

#include "fms_msgs/srv/command_robot.hpp"
#include "fms_msgs/srv/active_robot.hpp"
#include "fms_msgs/srv/submit_task.hpp"
#include "fms_msgs/srv/cancel_task.hpp"
#include "fms_msgs/srv/terminate.hpp"
#include "fms_msgs/srv/disable_region.hpp"

#include <eigen3/Eigen/Dense>
#include <sstream>
#include <fstream>
#include <chrono>
#include <thread>
#include <pthread.h>
#include <mutex>
#include <atomic>
#include <map>

#include <fleet_manager/utils.h>
#include <fleet_manager/topological_map.h>
#include <fleet_manager/multi_agent_status_monitor.h>
#include <fleet_manager/multi_agent_task_planner.h>
#include <fleet_manager/multi_agent_route_planner.h>
#include <fleet_manager/task_states.hpp>
#include <fleet_manager/robot_states.hpp>
#include <fleet_manager/route_table.h>

#include "fms_msgs/msg/route_info.hpp"
#include "fms_msgs/msg/curr_step.hpp"

using namespace std::chrono_literals;

namespace FMS
{
class FleetManager : public rclcpp::Node
{
public:

    FleetManager ();
    ~FleetManager();
    /* Service Server */
    rclcpp::Service<fms_msgs::srv::ActiveRobot>::SharedPtr srv_active_robot_server;     // Service of robot activation
    rclcpp::Service<fms_msgs::srv::CommandRobot>::SharedPtr srv_command_robot_server;    // Service of robot command
    rclcpp::Service<fms_msgs::srv::SubmitTask>::SharedPtr srv_submit_task_server;      // Service of task submit
    rclcpp::Service<fms_msgs::srv::CancelTask>::SharedPtr srv_cancel_task_server;      // Service of task cancel
    rclcpp::Service<fms_msgs::srv::Terminate>::SharedPtr srv_terminate_server;        // Service of program termination
    rclcpp::Service<fms_msgs::srv::DisableRegion>::SharedPtr srv_disable_region_server;   // Service of disable region


    /* Publishers */
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_topolomap_viz;               // For Rviz
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_topolomap_node_viz;               // For Rviz
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_routes_viz;                  // For Rviz
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_local_routes_viz;            // For Rviz
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_poses_viz;                   // For Rviz
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_task_viz;                    // For Rviz
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_temp_viz;                    // For Rviz
    rclcpp::Publisher<fms_msgs::msg::Tasks>::SharedPtr pub_task_table;              // Publish the task state table to GUI
    rclcpp::Publisher<fms_msgs::msg::Robots>::SharedPtr pub_robot_table;            // Publish the robot state table to GUI
    rclcpp::Publisher<fms_msgs::msg::RouteTable>::SharedPtr pub_route_table;        // Publish the route table to GUI

    std::vector<rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr> pub_routes;                 // Publisher for final routes
    std::vector<rclcpp::Publisher<fms_msgs::msg::RobotState>::SharedPtr> pub_robot_state;      // Publisher for robot state


    std::vector<rclcpp::Publisher<fms_msgs::msg::RouteInfo>::SharedPtr> pub_route_infos;

    /* Subscribers */
    std::vector<rclcpp::Subscription<fms_msgs::msg::RobotInfo>::SharedPtr> sub_robot_info;    // Subscriber for robot info

    std::vector<rclcpp::Subscription<fms_msgs::msg::CurrStep>::SharedPtr>  sub_curr_steps;
    std::vector<std::pair<int,int>> robot_curr_seg;


    /* Maps */
    TopologicalMap topolomap;                       // Topological Map
    bool is_directed_graph;                         // Directed Graph or Not
    bool remove_stations;                           // Remove station nodes
    double dist_bw_nodes;                           // Unit distance b/w nodes

    /* User Parameters */
    std::string filename_topolomap;                 // File name of topological map
    std::string filename_task_list;                 // File name of task list
    std::string filename_task_table;                // File name of task state table
    std::string filename_robot_table;               // File name of robot state table
    int num_of_robots;                              // Number of robots
    bool load_state_tables;                         // True: load state tables, False: set default tables
    double display_scale;                           // Visualization scale in rviz
    bool run_RHCR = false;                          // True: RHCR algorithm (Li et al. AAAI-2021), False: Normal MAPF algorithm
    double time_thr_task_planning  = 10.0;          // Task planning period
    double time_thr_route_planning = 20.0;          // Route replanning period
    double time_thr_monitor        = 3.0;           // Main iteration period
    int thr_delay = 10;                             // If a robot waits for this time step, its path needs to be updated

    std::string filename_result;
    std::string result_test_dir;
    int test_mode;
    int fms_mode;                                   // FMS mode [0: centralized FMS], [1: semi-centralized FMS], [2: De-centralized FMS]
    int optimizer;                                  // ADG optimizer [0: ADG], [1: ESES]


    /* Global Variables */
    bool running = true;                                        // True: currently running
    int iter_monitor = 0;                                       // Iteration number
    bool is_global_route_updated;                               // If true, global routes are updated
    bool need_path_replanning = true;                           // If ture, run MARP module
    bool need_task_replanning = true;                          // If ture, run MATP module
    bool ready_planning = false;                                // Ready for MARP module
    bool check_replanning;                                      // If ture, we regularly check the state update for replanning
    bool disable = false;                                       // ROS parameter to stop this program
    std::vector<bool> ready;                                    // Ready for MARP module (each robot)
    std::vector<std::string> robot_names;                       // Robot name
    std::map<std::string, int> robot_name_index;
    std::vector<std::string> robot_ids;                         // Robot ID
    std::vector<geometry_msgs::msg::Pose2D> robot_pose;              // Robot pose data
    std::vector<size_t> robot_info;                             // Subscribers for robot info msg
    std::vector<int> robot_steps;                               // Current robot step of local route (index of waypoint in local route)
    std::vector<std::vector<Eigen::Vector3d>> local_routes;     // Computed local routes
    std::vector<RobotAction> action_list;                       // Inserted action list
    int num_inserted_actions = 0;                               // Number of inserted asctions

    std::vector<std::pair<int,int>> robot_locs;

    rclcpp::Clock::SharedPtr clock;
    Util util;                      // Utility functions

    /*********************************/
    /*  Task/Robot State Tables and Route Table  */
    TaskStates*  task_states;       // Task state table
    RobotStates* robot_states;      // Robot state table
    RouteTable*  route_table;       // Route table
    /*********************************/

    /* Class for Multi Agent Status Monitor */
    MultiAgentStatusMonitor MASM;
    /* Class for Multi Agent Task Planner */
    MultiAgentTaskPlanner MATP;
    /* Class for Multi Agent Route Planner */
    MultiAgentRoutePlanner MARP;

    // --------------------------------------------------

    /* Callback Functions */
    bool robotActRobotSrvCallback(fms_msgs::srv::ActiveRobot::Request::SharedPtr req, fms_msgs::srv::ActiveRobot::Response::SharedPtr res);
    bool robotCmdSrvCallback(fms_msgs::srv::CommandRobot::Request::SharedPtr req, fms_msgs::srv::CommandRobot::Response::SharedPtr res);
    bool submitTaskSrvCallback(fms_msgs::srv::SubmitTask::Request::SharedPtr req, fms_msgs::srv::SubmitTask::Response::SharedPtr res);
    bool cancelTaskSrvCallback(fms_msgs::srv::CancelTask::Request::SharedPtr req, fms_msgs::srv::CancelTask::Response::SharedPtr res);
    bool terminateSrvCallback(fms_msgs::srv::Terminate::Request::SharedPtr req, fms_msgs::srv::Terminate::Response::SharedPtr res);
    // bool disableRegionSrvCallback(fms_msgs::DisableRegion::Request &req, fms_msgs::DisableRegion::Response &res);
    // void robotInfoCallback(const fms_msgs::msg::RobotInfo::SharedPtr msg);
    void robotInfoCallback(const fms_msgs::msg::RobotInfo::SharedPtr msg, int i);
    void robotStepCallback(const fms_msgs::msg::CurrStep::SharedPtr msg, int i);

    /* Init Map, Robot, Tasks */
    void initFMS();                         // Initialize FMS
    void initStateTables();                 // Initialize robot & task state tables
    void loadTopologicalMap();              // Load topological map
    void initTasks();                       // Initialize task table
    bool initTasksLoad();                   // Initialize task table by loading the task list
    void initTasksRandom();                 // Initialize task table based on random tasks
    void initRobots();                      // Initialize robot table

    /* Main Functions */
    bool runMultiAgentStatusMonitor();      // Update the task and robot state tables
    bool runMultiAgentTaskAssignment();     // Run multi agent task planner
    void runMultiAgentRoutePlanner();       // Plan multi-agent routes (global route)
    void runMultiAgentTrafficControl();     // Perform traffic control to compute local routes
    void publishRoute(std::vector<Eigen::Vector3d> _p, int _topic); // Publish the computed local routes

    void publishRouteInfo(std::vector<Eigen::Vector3d> _p, int _topic); // Publish the computed local routes
    void publishRobotState();                                       // Publish the robot state table
    void publishStateTables();                                      // Publish the task state table
    void publishRouteTables();                                      // Publish the route table

    /* MATP & MARP Loop Thread Functions */
    bool isReady();
    void runIteration();        // Main iteration function
    void Run();                                 // Start this thread
    void Loop();                                // Main loop of this thread
    void Release();                             // Release this thread
    // --------------------------------------------------

private:
	std::mutex mtx_lock;            // Mutex for shared variable
	std::thread tid;                // Thread ID
	std::atomic_bool condition;     // Running condition
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;


};
} // namespace fleet_manager
