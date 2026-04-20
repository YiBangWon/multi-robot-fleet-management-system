// Controller helper declarations for robot motion commands.

#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "robot_manager/dwa.h"

#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <eigen3/Eigen/Dense>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <memory>
#include <cmath>

namespace robot_manager {

class Controller
{
  public:

    Controller();
    void setPath(const std::vector<Eigen::Vector3d>& path);                     // Update the local route
    geometry_msgs::msg::Twist getCmdVel(Eigen::Vector3d pose_curr, float delta_t,    // Get velocity command
                                   Eigen::Vector2d vw);
    geometry_msgs::msg::Twist getCmdVel_Pure(Eigen::Vector3d pose);                  // Get velocity command from pure controller
    geometry_msgs::msg::Twist getCmdVel_DWA(Eigen::Vector3d pose);                  // Get velocity command from DWA controller
    void updatePathCounter(Eigen::Vector3d pose);                               // Update path counter: index of waypoint
    bool checkGoalArrival(Eigen::Vector3d pose);                                // Check goal arrival
    std::vector<Eigen::Vector3d> samplePath(Eigen::Vector3d start,              // Upsample the local path
                                            const std::vector<Eigen::Vector3d>& path);
    void extractVelocity(Eigen::Vector3d &pose1, Eigen::Vector3d &pose2,        // Compute linear and angular velocities
                         double dt, double &vx, double &vy, double &omega);
    int sign(double x);
    double normalize_theta(double theta);
    void setGtPose();                                                          // Set ground truth pose for DWA controller

    DWA* dwa;

    std::string robot_name;                   // Robot name for debug
    std::vector<Eigen::Vector3d> local_path;  // Local path to follow
    std::vector<Eigen::Vector2d> gt_pose;     // Ground truth pose
    Eigen::Vector3d pose_curr;                // Current robot pose
    int path_counter;                         // Index of path waypoint
    bool is_pure_rotation;                    // True: enable pure rotation
    float goal_range;                         // Goal range size
    double max_v;                             // Maximum linear velocity
    double max_w;                             // Maximum angular velocity
};

}

#endif
