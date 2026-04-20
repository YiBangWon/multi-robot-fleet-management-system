// Declarations for the single-robot controller node.

#ifndef FMS_SINGLE_CTRL_NODE_H
#define FMS_SINGLE_CTRL_NODE_H

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav2_msgs/action/navigate_through_poses.hpp>
#include <nav2_msgs/srv/clear_costmap_around_robot.hpp>
#include <nav2_msgs/srv/clear_entire_costmap.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

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
#include <vector>
#include <iostream>

namespace robot_manager {

class fms_single_ctrl_node : public rclcpp::Node {

public:
  fms_single_ctrl_node();
  ~fms_single_ctrl_node();

  // ROS Action Client
  rclcpp_action::Client<nav2_msgs::action::NavigateThroughPoses>::SharedPtr nav_act_client;

  // ROS Service Client
  rclcpp::Client<fms_msgs::srv::ActiveRobot>::SharedPtr actrobot_srv_client;   // ROS Service Client
  rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr global_clear_entire_Costmap_srv_client;     // Ros Clear Global Costmap Service Client

  // ROS Publishers
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmdvel;        // Publisher for cmd_vel
  rclcpp::Publisher<fms_msgs::msg::RobotInfo>::SharedPtr pub_robot_info;    // Publisher for robot_info

  // ROS Subscribers
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom;         // Subscriber for odometry
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_pose;         // Subscriber for odometry
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_route;         // Subscriber for route
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_path;         // Subscriber for route
  rclcpp::Subscription<fms_msgs::msg::RobotState>::SharedPtr sub_robot_state;  // Subscriber for robot_state

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_gt_pose;  // Subscriber for robot pose

  // Global Variables
  bool is_pure_rotation;             // true: enable pure rotation
  bool run_stage;
  double max_v;                      // Maximum linear velocity
  double max_w;                      // Maximum angular velocity
  std::string robot_name;            // Robot names
  geometry_msgs::msg::Pose2D robot_pose;  // Robot poses
  rclcpp::Time update_time;             // Last update time of robot pose
  fms_msgs::msg::RobotModel robot_model;  // Robot model
  bool ready;
  rclcpp::Clock::SharedPtr clock;

  /**************************************/
  /*  Robot state manager & controller  */
  RobotManager* manager;             // Check and update robot states
  Controller controller;             // Compute cmd_vel
  /**************************************/

  void Run();
  void checkGoalArrival();
  void sendActivationSrv();
  void sendClearmapSrv();
  void updateRobotState();  // Update robot states
  void publishRobotInfo();  // Publish robot information
  void subRobotStateCallback(const fms_msgs::msg::RobotState::SharedPtr msg);
  void subOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void subPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  void subRouteCallback(const nav_msgs::msg::Path::SharedPtr msg);
  void goal_response_callback(rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateThroughPoses>::SharedPtr goal_handle);
  void feedback_callback(
       rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateThroughPoses>::SharedPtr,
       const std::shared_ptr<const nav2_msgs::action::NavigateThroughPoses::Feedback> feedback);
  void result_callback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateThroughPoses>::WrappedResult &result);


  void send_request_entire_clear_global_();
  void subPathCallback(const nav_msgs::msg::Path::SharedPtr msg);
  Eigen::Vector3d getPoseVec(geometry_msgs::msg::Pose pose); // Get pose vector from pose msg

  void subGtPoseCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

private:

};

}

#endif
