// Robot-manager declarations for route execution and feedback publishing.

#ifndef FLEET_MANAGER_H
#define FLEET_MANAGER_H

#include "rclcpp/rclcpp.hpp"
#include <eigen3/Eigen/Dense>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <fms_msgs/msg/node.hpp>
#include <fms_msgs/msg/robots.hpp>
#include <fms_msgs/msg/robot_state.hpp>
#include <fms_msgs/msg/robot_info.hpp>

#include <cstdlib>
#include <ctime>

namespace robot_manager {

// Sate of assigned task
struct TaskState {
    std::string TaskID;
    fms_msgs::msg::Node GoalNode;
    Eigen::Vector3d target_pose;
    std::vector<Eigen::Vector3d> path;
    size_t progress;
};


class RobotManager {

public:

    RobotManager(std::string RobotID_, size_t RobotType_, rclcpp::Clock::SharedPtr clock_);

    // Insert the robot state_msg and update ctrl_mode
    void insert_state_msg(fms_msgs::msg::RobotState state_msg);
    // Update robot ctrl_mode by checking the crruent state
    void update_state(geometry_msgs::msg::Pose2D pose_);

    bool check_goal_arrival();          // Check goal arrival
    bool check_task_completion();       // Check task completion: simply check processing time
    bool check_battery_charging();      // Check battery capacity (need to charging?)
    bool check_charging_completion();   // Check charging completion based on thr_max_battery_percent
    bool check_waiting_completion();    // Check waiting completion based on thr_waiting_time
    bool check_error();
    void update_battery_capacity();     // Update virtual battery capacity

    // Robot id and type
    std::string RobotID;
    size_t RobotType;

    // Robot & Task state and mode
    TaskState task_state;
    size_t robot_mode;      // External robot mode (Subscribe from FMS: fms_msgs::RobotState::mode)
    size_t ctrl_mode;       // Inertial robot mode (Publish to FMS: fms_msgs::RobotInfo::ctrl_mode)

    // Current robot location and bettery
    Eigen::Vector3d curr_pose;
    size_t battery_percent;
    rclcpp::Time last_update_time;
    rclcpp::Time battery_update_time;
    rclcpp::Clock::SharedPtr clock;
    bool is_goal_arrival;

    // Robot Parameters
    size_t thr_min_battery_percent = 30;    // 20
    size_t thr_max_battery_percent = 80;
    double thr_task_time = 5.0;
    double thr_battery_inc_time = 1.0;      // 0.2
    double thr_battery_dec_time = 100.0;     // 20.0
    double thr_waiting_time = 5.0;
    float goal_range = 0.3;

};

}

#endif // FLEET_MANAGER_H
