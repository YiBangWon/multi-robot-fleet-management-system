// Dynamic-window local planner declarations.

#ifndef DWA_H
#define DWA_H

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <iostream>
#include <fstream>
#include <vector>
#include <array>
#include <cmath>
#include <limits>
#include <utility>
#include <eigen3/Eigen/Dense>

namespace robot_manager {

// Robot type
enum class RobotType {Circle, Rectangle};

class DWA
{
  public:
    struct Config {
        //     
        double max_speed            = 0.30;        // [m/s]  (    )
        double min_speed            = 0.00;        //     0.30 
        double max_yaw_rate         = M_PI * 1.8;  // [rad/s]  100 deg/s
        double max_accel            = 2.0;         // [m/s]   1  1.2 m/s  
        double max_delta_yaw_rate   = M_PI * 1.2;  // [rad/s]  70 deg/s

        //    &  
        double v_resolution         = 0.01;        // [m/s]   0.05  0.90  19 
        double yaw_rate_resolution  = M_PI / 60;   // [rad]   3 deg
        double dt                   = 0.10;        // [s]       = -   
        double predict_time         = 2.0;         // [s]        1.5~2 s  

        //    
        double to_goal_cost_gain    = 0.25;        //   
        double speed_cost_gain      = 0.50;        //   (0.3~0.5 )
        double obstacle_cost_gain   = 0.8;         //    ( )
        double robot_stuck_flag_cons= 0.001;       //  

        //    
        RobotType robot_type        = RobotType::Circle;
        double     robot_radius     = 0.25;        // [m]    +  3~5 cm
        double     robot_width      = 0.50;        // (Rectangle    )
        double     robot_length     = 0.70;

        std::vector<std::vector<std::array<double, 2>>> ob;   // Obstacles in 2 types (fixed and dynamics)
    };

    using State = std::array<double,5>;
    using Trajectory = std::vector<State>;

    DWA();
    // Compute control and trajectory given current state and goal
    std::pair<std::array<double,2>, Trajectory> plan(const State &x, const std::array<double,2> &goal);
    std::vector<Eigen::Vector2d> gt_pose;     // Ground truth pose

  private:
    Config cfg_;
    State motion(const State &x, const std::array<double,2> &u);
    std::array<double, 4> calcDynamicWindow(const State &x);
    Trajectory predictTrajectory(const State &x_init, double v, double y);
    double calcObstacleCost(const Trajectory &traj);
    double calcToGoalCost(const Trajectory &traj, const std::array<double,2> &goal);
    std::pair<std::array<double, 2>, Trajectory> calcControlAndTrajectory(const State &x, const std::array<double, 4> &dw, const std::array<double, 2> &goal);
    void loadObstacles(const std::string& file_path);
    void setObstacle();
};
}

#endif
