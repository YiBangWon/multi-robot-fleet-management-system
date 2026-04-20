// Task-state table used to track queued, active, and completed task groups.

#ifndef TaskStates_H
#define TaskStates_H

#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <fstream>      // std::ifstream
#include <eigen3/Eigen/Dense>
#include <vector>

#include <fms_msgs/msg/tasks.hpp>
#include <fms_msgs/msg/node.hpp>
#include <fms_msgs/msg/task_info.hpp>
#include <fms_msgs/msg/task_state.hpp>
#include <yaml-cpp/yaml.h>

namespace FMS
{

  class TaskUnit    // Single subtask
  {
    public:
    TaskUnit(){};
    TaskUnit(Eigen::Vector3d location_, std::string node_name_){
        location = location_;
        node_name = node_name_;
        node_id = -1;
    };
    Eigen::Vector3d location;   // Task unit location
    std::string node_name;      // Node name for task unit
    int node_id;                // Node ID for task unit
  };

  class TaskStates
  {
  public:

    /// Default constructor
    TaskStates(){num_tasks = 0;};

    class Task
    {
        public:

        Task(){
          submission_time = rclcpp::Time(0); // rclcpp::Time::now();
          start_time = rclcpp::Time(0);
          end_time = rclcpp::Time(0);
          remaining_time = rclcpp::Time(0);
          processing_time = rclcpp::Time(0);
          curr_task_index = 0;
        };

        Task(int task_id_, int robot_id_, std::vector<TaskUnit> task_units_){
            task_id        = task_id_;
            robot_id       = robot_id_;
            task_units     = task_units_;
            curr_task_loc  = task_units[0].location;
            curr_task_node = task_units[0].node_name;
            state = fms_msgs::msg::TaskState::STATE_QUEUED;
        };

        // Inertial variables
        int task_id;                        // task id
        int robot_id;                       // allocated robot id
        size_t state;                       // current state
        size_t priority;                    // task priority
        std::vector<TaskUnit> task_units;   // Set of task_units (subtasks)
        int curr_task_index;                // Index of current task_unit (subtask)
        Eigen::Vector3d curr_task_loc;      // Location of current task_unit (subtask)
        std::string curr_task_node;         // Node str id of current task_unit (subtask)

        // External variables
        std::string TaskID;             // String task ID
        std::string RobotID;            // String robot ID
        size_t Type;                    // Task type

        // Time variables
        rclcpp::Time remaining_time;       // Remained time for task completion
        rclcpp::Time processing_time;      // Current task processing time
        rclcpp::Time submission_time;      // Task insertion time
        rclcpp::Time start_time;           // Starting time
        rclcpp::Time end_time;             // Ending time
    };

    void insertTask(int task_id_, int robot_id_, std::vector<TaskUnit> task_units_){
        TaskStates::Task task(task_id_, robot_id_, task_units_);
        table.push_back(task);
        num_tasks = table.size();
    }

    void insertTask(TaskStates::Task task_){
        table.push_back(task_);
        num_tasks = table.size();
    }

      TaskStates::Task insertNewTask(std::vector<TaskUnit> task_units, size_t type, size_t priority){

            TaskStates::Task task;
            task.task_id         = num_tasks + 1;
            task.robot_id        = -1;
            task.task_units      = task_units;
            if (type == fms_msgs::msg::TaskInfo::TYPE_ORDERPICK)
                task.TaskID      = "OP" + std::to_string(task.task_id);
            else if (type == fms_msgs::msg::TaskInfo::TYPE_STACKING)
                task.TaskID      = "ST" + std::to_string(task.task_id);
            task.Type            = type;
            task.curr_task_loc   = task_units[0].location;
            task.curr_task_node  = task_units[0].node_name;
            task.RobotID         = "";
            task.state           = fms_msgs::msg::TaskState::STATE_QUEUED;
            task.priority        = priority;
            // task.submission_time = ros::Time::now();
            table.push_back(task);
            num_tasks = table.size();
            return task;
    }

    bool cancelTask(std::string task_id){
        for (int i = 0; i < num_tasks; i++){
            if (table[i].TaskID == task_id){
                if (table[i].state == fms_msgs::msg::TaskState::STATE_QUEUED){
                    // Program can cancel a task with state of STATE_QUEUED only
                    table[i].state = fms_msgs::msg::TaskState::STATE_CANCELED;
                    return true;
                }
                else {
                    return false;
                }
            }
        }
        return false;
    }

    void updateTask(int task_id_, int robot_id_, size_t state_, std::string RobotID_){

        if (task_id_ < 0) return;

        table[task_id_].state = state_;
        table[task_id_].robot_id = robot_id_;
        table[task_id_].RobotID = RobotID_;

        rclcpp::Clock clock;

        if (state_ == fms_msgs::msg::TaskState::STATE_QUEUED)
          table[task_id_].start_time = rclcpp::Time(0,0);
        else if (state_ == fms_msgs::msg::TaskState::STATE_ACTIVE)
          table[task_id_].start_time = clock.now();   // Now
        else if (state_ == fms_msgs::msg::TaskState::STATE_COMPLETED){
          table[task_id_].end_time        = clock.now(); // Now
          table[task_id_].remaining_time  = rclcpp::Time(0,0);
          table[task_id_].processing_time = rclcpp::Time((table[task_id_].end_time - table[task_id_].start_time).seconds(), 0);
        }
        else if (state_ == fms_msgs::msg::TaskState::STATE_FAILED){
          table[task_id_].start_time      = rclcpp::Time(0,0);
          table[task_id_].end_time        = rclcpp::Time(0,0);
          table[task_id_].remaining_time  = rclcpp::Time(0,0);
          table[task_id_].processing_time = rclcpp::Time(0,0);
        }
    }

    void updateNextTaskUnit(int task_id){
        table[task_id].curr_task_index++;
        TaskUnit current_task = table[task_id].task_units[table[task_id].curr_task_index];
        table[task_id].curr_task_loc = current_task.location;
        table[task_id].curr_task_node = current_task.node_name;
    }

    bool isLastTaskUnit(int task_id){
        if (table[task_id].curr_task_index == table[task_id].task_units.size() - 1){
            return true;
        }
        return false;
    }

    fms_msgs::msg::Tasks getTaskStateTable(){
        // std::cout << "----------------- Task State Table ------------------" << std::endl;
        fms_msgs::msg::Tasks tasks;
        for (int i = 0; i < (int)table.size(); i++){
            if (table[i].state == fms_msgs::msg::TaskState::STATE_CANCELED)
                continue;
            fms_msgs::msg::TaskState task;
            task.task_id               = table[i].TaskID;
            task.start_time            = table[i].start_time;
            task.end_time              = table[i].end_time;
            task.processing_time       = table[i].processing_time;
            task.remaining_time        = table[i].remaining_time;
            task.info.submission_time  = table[i].submission_time;
            task.state                 = table[i].state;
            task.robot_id              = table[i].RobotID;
            task.info.type             = table[i].Type;
            task.info.priority         = table[i].priority;
            int num_tasks = std::min((int)table[i].task_units.size(), num_viz_tasks);
            for (int j = 0; j < num_tasks; j++){
                fms_msgs::msg::Node node;
                node.name = table[i].task_units[j].node_name;
                node.pose.x = table[i].task_units[j].location[0];
                node.pose.y = table[i].task_units[j].location[1];
                node.pose.theta = table[i].task_units[j].location[2];
                task.info.task_nodes.push_back(node);
            }
            tasks.tasks.push_back(task);
            if (task.state != 0){
                // std::cout << "[" << task.task_id << " " << task.robot_id << " State-" << task.state << "] ";
            }
        }
        // std::cout << std::endl;
        // std::cout << "-----------------------------------------------------" << std::endl;
        return tasks;
    }

    void saveTaskStateTable(std::string file_name){
        YAML::Emitter tasks_out;
        std::ofstream tasks_fout;
        tasks_fout.open(file_name.c_str());
        for (int i = 0; i < num_tasks; i++){
            tasks_out << YAML::BeginMap;
            tasks_out << YAML::Key << "task_id";
            tasks_out << YAML::Value << table[i].task_id;
            tasks_out << YAML::Key   << "robot_id";
            tasks_out << YAML::Value << table[i].robot_id;
            tasks_out << YAML::Key   << "state";
            tasks_out << YAML::Value << table[i].state;
            tasks_out << YAML::Key   << "priority";
            tasks_out << YAML::Value << table[i].priority;
            tasks_out << YAML::Key   << "TaskID";
            tasks_out << YAML::Value << table[i].TaskID;
            tasks_out << YAML::Key   << "RobotID";
            tasks_out << YAML::Value << table[i].RobotID;
            tasks_out << YAML::Key   << "Type";
            tasks_out << YAML::Value << table[i].Type;
            tasks_out << YAML::Key   << "submission_time";
            std::stringstream submission_time_ss;
            submission_time_ss << table[i].submission_time.seconds() << "." << table[i].submission_time.nanoseconds();
            tasks_out << YAML::Value << submission_time_ss.str();
            tasks_out << YAML::Key   << "start_time";
            std::stringstream start_time_ss;
            start_time_ss << table[i].start_time.seconds() << "." << table[i].start_time.nanoseconds();
            tasks_out << YAML::Value << start_time_ss.str();
            tasks_out << YAML::Key   << "end_time";
            std::stringstream end_time_ss;
            end_time_ss << table[i].end_time.seconds() << "." << table[i].end_time.nanoseconds();
            tasks_out << YAML::Value << end_time_ss.str();
            tasks_out << YAML::EndMap;
        }
        tasks_fout << tasks_out.c_str();
        tasks_fout.close();
    }


    void loadTaskStateTable(std::string file_name){
        // HERE
        clearTasks();
        std::vector<YAML::Node> tasks_nodes = YAML::LoadAllFromFile(file_name);
        for (int i = 0; i < tasks_nodes.size(); i++){
            Task task;
            task.task_id = tasks_nodes[i]["task_id"].as<int>();
            task.robot_id = tasks_nodes[i]["robot_id"].as<int>();
            task.state = tasks_nodes[i]["state"].as<int>();
            task.priority = tasks_nodes[i]["priority"].as<int>();
            task.TaskID = tasks_nodes[i]["TaskID"].as<std::string>();
            task.RobotID = tasks_nodes[i]["RobotID"].as<std::string>();
            task.Type = tasks_nodes[i]["Type"].as<int>();
            task.submission_time = rclcpp::Time(tasks_nodes[i]["submission_time"].as<double>());
            task.start_time = rclcpp::Time(tasks_nodes[i]["start_time"].as<double>());
            task.end_time = rclcpp::Time(tasks_nodes[i]["end_time"].as<double>());
            table.push_back(task);
        }
        num_tasks = table.size();

    }


    void clearTasks(){
        num_tasks = 0;
        table.clear();
    }

    int num_tasks;
    int num_viz_tasks = 6;
    std::vector<Task> table;

  };

}

#endif

