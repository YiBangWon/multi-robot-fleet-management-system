// Declarations for the multi-robot controller node.

#ifndef FMS_CTRL_NODE_H
#define FMS_CTRL_NODE_H

#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <robot_manager/robot_manager.h>
#include <robot_manager/controller.h>

#include "fms_msgs/msg/robot_state.hpp"
#include "fms_msgs/msg/task_state.hpp"
#include "fms_msgs/msg/robot_info.hpp"
#include "fms_msgs/msg/robot_model.hpp"
#include "fms_msgs/srv/active_robot.hpp"

#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <eigen3/Eigen/Dense>
#include <memory>
#include <string>
#include <algorithm>
#include <map>

namespace robot_manager {

class fms_ctrl_node : public rclcpp::Node {

public:
  fms_ctrl_node();

  // ROS Service Client
  rclcpp::Client<fms_msgs::srv::ActiveRobot>::SharedPtr actrobot_srv_client; // ROS Service Client

  // ROS Publishers
  std::vector<rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr> pub_cmdvel; // Publisher for cmd_vel
  std::vector<rclcpp::Publisher<fms_msgs::msg::RobotInfo>::SharedPtr> pub_robot_info;  // Publisher for robot_info

  // ROS Subscribers
  std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> sub_odom;    // Subscriber for odometry
  std::vector<rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr> sub_path;    // Subscriber for route
  std::vector<rclcpp::Subscription<fms_msgs::msg::RobotState>::SharedPtr> sub_robot_state;    // Subscriber for robot_state

  // Global Variables
  int nr_of_robots;                               // Number of robots
  bool is_pure_rotation;                          // true: enable pure rotation
  double max_v;                                   // Maximum linear velocity
  double max_w;                                   // Maximum angular velocity
  std::vector<std::string> robot_names;           // Robot names
  std::vector<geometry_msgs::msg::Pose2D> robot_pose;  // Robot poses
  std::vector<rclcpp::Time> update_time;             // Last update time of robot pose
  fms_msgs::msg::RobotModel robot_model;               // Robot model
  std::vector<bool> ready;
  rclcpp::Clock::SharedPtr clock;
  std::map<std::string, int> robot_name_index;

  /**************************************/
  /*  Robot state manager & controller  */
  std::vector<RobotManager> managers;            // Check and update robot states
  std::vector<Controller> controller;            // Compute cmd_vel
  /**************************************/

  void Run();
  void sendActivationSrv();
  void updateRobotState();  // Update robot states
  void publishRobotInfo();  // Publish robot information
  void checkGoalArrival();

  // Callback functions for subscribers
  void subRobotStateCallback(const fms_msgs::msg::RobotState::SharedPtr msg, int index);
  void subOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg, int index);
  void subPathCallback(const nav_msgs::msg::Path::SharedPtr msg, int index);
  Eigen::Vector3d getPoseVec(geometry_msgs::msg::Pose pose); // Get pose vector from pose msg
  
};

}

#endif
