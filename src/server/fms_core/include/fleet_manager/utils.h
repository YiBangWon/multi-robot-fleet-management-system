// Declarations for shared visualization and helper utilities.


#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <fstream>
#include <fleet_manager/topological_map.h>
#include <fleet_manager/task_states.hpp>
#include <fleet_manager/robot_states.hpp>
#include <fleet_manager/route_table.h>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace FMS
{

class Util
{
    public:
        void setStateTables(TaskStates* task_states_, RobotStates* robot_states_){task_states = task_states_; robot_states = robot_states_;};
        void plotRouteTable(std::vector<std::vector<NodeRT*>> route_table, rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_);
        void plotLocalRoutes(std::vector<std::vector<Eigen::Vector3d>> routes, rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_);
        void plotTopologicalMap(const TopologicalMap& topolomap, rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_);
        void plotPoses(std::vector<geometry_msgs::msg::Pose2D> poses_, std::vector<std::string> robot_ids, rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_);
        void plotTasks(rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_);
        void openLogFiles();
        void setColor(int num_of_robots);
        float getYaw(const geometry_msgs::msg::Quaternion &_rot);

        rclcpp::Clock::SharedPtr clock;

        TaskStates*  task_states;       // Task state table
        RobotStates* robot_states;      // Robot state table

        std::vector<Eigen::Vector3d> robot_color;
        double display_scale = 1.0;
        bool color_path = true;

        void plotNodes(const std::vector<Eigen::Vector2d>& nodes, const std::vector<int>& node_ids, rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub);
};

int getMinIndex(std::vector<double> vec);
int getMaxIndex(std::vector<double> vec);

}
