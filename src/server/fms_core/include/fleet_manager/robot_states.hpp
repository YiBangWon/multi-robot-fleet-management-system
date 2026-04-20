// Robot-state table used to track execution status, assignments, and replanning flags.

#ifndef RobotStates_H
#define RobotStates_H

#include <rclcpp/rclcpp.hpp>
// #include <tf/tf.h>
// #include <geometry_msgs/msg/PoseWithCovariance.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/msg/pose_with_covariance.hpp>
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <vector>
#include "fms_msgs/msg/robots.hpp"
#include "fms_msgs/msg/robot_state.hpp"
#include "fms_msgs/msg/robot_model.hpp"
#include "fms_msgs/srv/command_robot.hpp"


#include <yaml-cpp/yaml.h>

namespace FMS
{

  class RobotAction
  {
  public:
    RobotAction(){};

    RobotAction(int num, size_t action_, std::string robot_id_, std::string goal_node_){
        action = action_;
        robot_id = robot_id_;
        goal_node = goal_node_;
        if (action_ == (size_t)fms_msgs::srv::CommandRobot::Request::ACTION_IDLE){
            action_id = "IDLE_" + std::to_string(num);
        }
        else if (action_ == (size_t)fms_msgs::srv::CommandRobot::Request::ACTION_MOVING){
            action_id = "MOVING_" + std::to_string(num);
        }
        else if (action_ == (size_t)fms_msgs::srv::CommandRobot::Request::ACTION_CHARGING){
            action_id = "CHARGING_" + std::to_string(num);
        }
        else if (action_ == (size_t)fms_msgs::srv::CommandRobot::Request::ACTION_WAITING){
            action_id = "WAITING" + std::to_string(num);
        }
        else if (action_ == (size_t)fms_msgs::srv::CommandRobot::Request::ACTION_PAUSE){
            action_id = "PAUSE" + std::to_string(num);
        }
        else if (action_ == (size_t)fms_msgs::srv::CommandRobot::Request::ACTION_RESUME){
            action_id = "RESUME" + std::to_string(num);
        }
    };

    std::string action_id;
    size_t action;
    std::string robot_id;
    std::string goal_node;
  };


  class RobotStates
  {
  public:

    /// Default constructor
    RobotStates(){};

    class Robot
    {
      public:
      Robot(){};

      // Inertial variables
      int robot_id;                             // robot id
      int task_id;                              // allocated task id
      size_t mode;                              // current mode
      Eigen::Vector3d pos;                      // current pose
      Eigen::Vector2d loc;                      // current node location
      Eigen::Vector3d target;                   // target
      std::vector<Eigen::Vector3d> path;        // following path
      size_t counter;
      size_t progress;
      int time_delay;
      size_t battery_percent;
      bool need_planning;
      fms_msgs::msg::RobotModel robot_model;

      // External variables
      std::string RobotID;
      std::string TaskID;
      std::string GoalNode;
      size_t Type = fms_msgs::msg::RobotState::TYPE_AGF;
      size_t Info;

      std::pair<int, size_t> prev_state; // Save previous state for pause action;
    };

    int insertRobot(RobotStates::Robot robot_){
        int index = num_robots;
        table.push_back(robot_);
        num_robots = table.size();
        return index;
    }


    // int insertRobot(fms_msgs::RobotModel robot_model_ = fms_msgs::RobotModel()){
    int insertRobot(fms_msgs::msg::RobotModel robot_model_){
        int index = num_robots;
        RobotStates::Robot robot;
        robot.robot_id          = index;
        robot.task_id           = -1;
        robot.mode              = fms_msgs::msg::RobotState::MODE_IDLE;
        robot.pos               = Eigen::Vector3d(0,0,0);
        robot.target            = Eigen::Vector3d(0,0,0);
        robot.counter           = 0;
        robot.progress          = 0;
        robot.time_delay        = 0;
        robot.battery_percent   = 100;
        robot.TaskID            = "";
        robot.GoalNode          = "";
        robot.Type              = robot_model_.type;
        robot.robot_model       = robot_model_;
        if (robot.Type == 0)
            robot.RobotID           = "AGF" + std::to_string(index);
        else if (robot.Type == 1)
            robot.RobotID           = "AGV" + std::to_string(index);
        else if (robot.Type == 2)
            robot.RobotID           = "AMR" + std::to_string(index);
        robot.need_planning     = false;
        table.push_back(robot);
        num_robots = table.size();
        std::cout << "robot.Type: " << robot.Type << std::endl;
        return index;
    }

    void updateCounter(int robot_id_){

        Eigen::Vector3d pose = table[robot_id_].pos;
        double min_dist = 9999;
        int index = 0;
        for (int i = 0; i < table[robot_id_].path.size(); i++){
            double dist = (table[robot_id_].path[i].head(2) - pose.head(2)).norm();
            if (dist < min_dist){
            min_dist = dist;
            index = i;
            }
        }
        table[robot_id_].counter = index;
        if (table[robot_id_].mode == fms_msgs::msg::RobotState::MODE_WAITING      ||
            table[robot_id_].mode == fms_msgs::msg::RobotState::MODE_CHARGING     ||
            table[robot_id_].mode == fms_msgs::msg::RobotState::MODE_ERROR        ||
            table[robot_id_].mode == fms_msgs::msg::RobotState::MODE_IDLE         ||
            table[robot_id_].mode == fms_msgs::msg::RobotState::MODE_PAUSED       ||
            table[robot_id_].mode == fms_msgs::msg::RobotState::MODE_TASK_PROCESSING ){
            table[robot_id_].time_delay = 0;
        }
        else if ((table[robot_id_].loc - pose.head(2)).norm() < 1.0){
            table[robot_id_].time_delay++;  // The robot is not moving
        }
        else {
            table[robot_id_].time_delay = 0;
            table[robot_id_].loc = pose.head(2);
        }
        // Update the progress
        int path_size = std::max((int)table[robot_id_].path.size()-1, 1);
        int progress = std::min(100, (int)(table[robot_id_].counter * 100 / path_size));
        table[robot_id_].progress = std::max(progress, (int)table[robot_id_].progress);
    }

    void clearAllCounter(){
      for (int i = 0 ; i < num_robots ; i++){
          table[i].counter = 0;
          table[i].time_delay = 0;
      }
    }

    void clearAllDelay(){
      for (int i = 0 ; i < num_robots ; i++){
          table[i].time_delay = 0;
      }
    }

    void clearAllNeedPlanning(){
      for (int i = 0 ; i < num_robots ; i++){
          table[i].need_planning = false;
      }
    }

    void clearCounter(int robot_id){
        table[robot_id].counter = 0;
        table[robot_id].time_delay = 0;
    }

    void incrementDelay(int robot_id_){
        table[robot_id_].time_delay++;
    }

    int getCounter(int robot_id_){
        return table[robot_id_].counter;
    }

    int getTimeDelay(int robot_id_){
        return table[robot_id_].time_delay;
    }

    bool getNeedPlanning(int robot_id_){
        return table[robot_id_].need_planning;
    }

    std::vector<bool> getNeedPlanningAll(){
        std::vector<bool> vec;
        for (int i = 0; i < num_robots; i++){
            vec.push_back(table[i].need_planning);
        }
        return vec;
    }

    void updateNeedPlanningAll(int thr_delay){
        for (int i = 0; i < num_robots; i++){
            if (table[i].time_delay > thr_delay){ // If a robot is stopped very long time
                table[i].need_planning = true;
            }
        }
    }

    void setNeedPlanning(int robot_id_, bool flag){
        table[robot_id_].need_planning = flag;
    }

    int getDistToTarget(int robot_id_){
        return (table[robot_id_].pos.head(2) - table[robot_id_].target.head(2)).norm();
    }

    void setPose(int robot_id_, Eigen::Vector3d pose_){
        table[robot_id_].pos = pose_;
    }


    void setPose(int robot_id_, geometry_msgs::msg::Pose2D pose_){
        table[robot_id_].pos[0] = pose_.x;
        table[robot_id_].pos[1] = pose_.y;
        table[robot_id_].pos[2] = pose_.theta;
    }


    void setPose(int robot_id_, geometry_msgs::msg::PoseWithCovariance pose_){
        double roll, pitch, yaw;
        tf2::Quaternion q ( pose_.pose.orientation.x, pose_.pose.orientation.y, pose_.pose.orientation.z, pose_.pose.orientation.w );
        tf2::Matrix3x3 m(q);
        m.getRPY ( roll, pitch, yaw );
        Eigen::Vector3d p ( pose_.pose.position.x, pose_.pose.position.y, yaw );
        table[robot_id_].pos = p;
    }

    void setPaths(std::vector<std::vector<Eigen::Vector3d>> paths){
        for (int i = 0; i < table.size(); i++){
            table[i].path.clear();
            table[i].path = paths[i];
            table[i].counter = 0;
        }
    }

    Eigen::Vector3d getPose(int robot_id_){
        return table[robot_id_].pos;
    }

    int getTaskID(int robot_id_){
        return table[robot_id_].task_id;
    }

    size_t getMode(int robot_id_){
        return table[robot_id_].mode;
    }

    int getRobotIndex(std::string robot_name){
        int index = -1;
        for (int i = 0; i <table.size(); i++){
            if (table[i].RobotID == robot_name)
            index = i;
        }
        return index;
    }


    void updateRobot(int robot_id_, int task_id_, size_t mode_, Eigen::Vector3d target_, std::string TaskID_, std::string GoalNode_){

        table[robot_id_].task_id    = task_id_;
        table[robot_id_].mode       = mode_;
        table[robot_id_].path.clear();
        table[robot_id_].target     = target_;
        table[robot_id_].counter    = 0;
        table[robot_id_].progress   = 0;
        table[robot_id_].time_delay = 0;
        table[robot_id_].TaskID     = TaskID_;
        table[robot_id_].GoalNode   = GoalNode_;
    }

    fms_msgs::msg::Robots getRobotStateTable(){
        // std::cout << "----------------- Robot State Table ------------------" << std::endl;
        // std::cout << (int)table.size() << std::endl;
        fms_msgs::msg::Robots robots;
        for (int i = 0; i < (int)table.size(); i++){
            fms_msgs::msg::RobotState robot;
            robot.robot_id          = table[i].RobotID;
            robot.task_id           = table[i].TaskID;
            robot.type              = table[i].Type;
            robot.progress          = table[i].progress;
            robot.battery           = table[i].battery_percent;
            robot.goal_node.name    = table[i].GoalNode;
            robot.mode              = table[i].mode;
            robot.target_pose.x     = table[i].target[0];
            robot.target_pose.y     = table[i].target[1];
            robot.target_pose.theta = table[i].target[2];
            robots.robots.push_back(robot);
            // std::cout << "[" << robot.robot_id << " " << robot.task_id << " Mode-" << robot.mode << "] ";
        }
        // std::cout << std::endl;
        // std::cout << "-----------------------------------------------------" << std::endl;
        return robots;
    }

    void saveRobotStateTable(std::string file_name){
        YAML::Emitter robots_out;
        std::ofstream robots_fout;
        robots_fout.open(file_name.c_str());
        for (int i = 0; i < num_robots; i++){
            robots_out << YAML::BeginMap;
            robots_out << YAML::Key   << "robot_id";
            robots_out << YAML::Value << table[i].robot_id;
            robots_out << YAML::Key   << "task_id";
            robots_out << YAML::Value << table[i].task_id;
            robots_out << YAML::Key   << "mode";
            robots_out << YAML::Value << table[i].mode;
            robots_out << YAML::Key   << "pos_x";
            robots_out << YAML::Value << table[i].pos[0];
            robots_out << YAML::Key   << "pos_y";
            robots_out << YAML::Value << table[i].pos[1];
            robots_out << YAML::Key   << "pos_z";
            robots_out << YAML::Value << table[i].pos[2];
            robots_out << YAML::Key   << "target_x";
            robots_out << YAML::Value << table[i].target[0];
            robots_out << YAML::Key   << "target_y";
            robots_out << YAML::Value << table[i].target[1];
            robots_out << YAML::Key   << "target_z";
            robots_out << YAML::Value << table[i].target[2];
            robots_out << YAML::Key   << "RobotID";
            robots_out << YAML::Value << table[i].RobotID;
            robots_out << YAML::Key   << "TaskID";
            robots_out << YAML::Value << table[i].TaskID;
            robots_out << YAML::Key   << "GoalNode";
            robots_out << YAML::Value << table[i].GoalNode;
            robots_out << YAML::Key   << "Type";
            robots_out << YAML::Value << table[i].Type;
            robots_out << YAML::Key   << "Info";
            robots_out << YAML::Value << table[i].Info;
            robots_out << YAML::Key   << "prev_state_first";
            robots_out << YAML::Value << table[i].prev_state.first;
            robots_out << YAML::Key   << "prev_state_second";
            robots_out << YAML::Value << table[i].prev_state.second;
            robots_out << YAML::EndMap;
        }
        robots_fout << robots_out.c_str();
        robots_fout.close();
    }

    void loadRobotStateTable(std::string file_name){
        num_robots = 0;
        table.clear();
        std::vector<YAML::Node> robots_nodes = YAML::LoadAllFromFile(file_name);
        for (int i = 0; i < robots_nodes.size(); i++){
            Robot robot;
            robot.robot_id = robots_nodes[i]["robot_id"].as<int>();
            robot.task_id = robots_nodes[i]["task_id"].as<int>();
            robot.mode = robots_nodes[i]["mode"].as<int>();
            robot.pos[0] = robots_nodes[i]["pos_x"].as<double>();
            robot.pos[1] = robots_nodes[i]["pos_y"].as<double>();
            robot.pos[2] = robots_nodes[i]["pos_z"].as<double>();
            robot.target[0] = robots_nodes[i]["target_x"].as<double>();
            robot.target[1] = robots_nodes[i]["target_y"].as<double>();
            robot.target[2] = robots_nodes[i]["target_z"].as<double>();
            robot.counter = 0;
            robot.progress = 0;
            robot.time_delay = 100;
            robot.battery_percent = 100;
            robot.RobotID = robots_nodes[i]["RobotID"].as<std::string>();
            robot.TaskID = robots_nodes[i]["TaskID"].as<std::string>();
            robot.GoalNode = robots_nodes[i]["GoalNode"].as<std::string>();
            robot.Type = robots_nodes[i]["Type"].as<int>();
            robot.Info = robots_nodes[i]["Info"].as<int>();
            robot.prev_state.first = robots_nodes[i]["prev_state_first"].as<int>();
            robot.prev_state.second = robots_nodes[i]["prev_state_second"].as<int>();

            if (robot.mode == fms_msgs::msg::RobotState::MODE_TASK_PROCESSING){
                robot.mode == fms_msgs::msg::RobotState::MODE_MOVING_TASK;
            }
            else if (robot.mode == fms_msgs::msg::RobotState::MODE_CHARGING){
                robot.mode == fms_msgs::msg::RobotState::MODE_MOVING_CHARGING;
            }
            else if (robot.mode == fms_msgs::msg::RobotState::MODE_WAITING){
                robot.mode == fms_msgs::msg::RobotState::MODE_MOVING_WAITING;
            }
            table.push_back(robot);
        }
        num_robots = table.size();
    }

    int num_robots = 0;
    std::vector<Robot> table;           // Robot state table
    // std::vector<bool> need_planning;    // Robot flags for path planning

  };

}

#endif

