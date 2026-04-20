// Status-monitoring module that keeps robot and task tables synchronized with execution feedback.

#include <iostream>

#include <fleet_manager/multi_agent_status_monitor.h>

namespace  FMS
{
    /* Initialize the MASM module */
    void MultiAgentStatusMonitor::initMASM(TaskStates* task_states_, RobotStates* robot_states_, TopologicalMap* topolomap_){
        task_states  = task_states_;                // Task state table
        robot_states = robot_states_;               // Robot state table
        num_of_robots = robot_states->num_robots;   // Number of robots
        updated_robot.resize(num_of_robots, true);  // Flags for updated robots
        topolomap = topolomap_;                     // Topological map
    }

    /* Check errors on currently assigned tasks */
    bool MultiAgentStatusMonitor::checkTaskErros(){

        for (int i = 0; i < num_of_robots; i++){
            int task_id = robot_states->table[i].task_id;
            if (task_id < 0) continue;
            if (task_states->table[task_id].state == fms_msgs::msg::TaskState::STATE_COMPLETED ||
                task_states->table[task_id].state == fms_msgs::msg::TaskState::STATE_FAILED    ||
                task_states->table[task_id].state == fms_msgs::msg::TaskState::STATE_CANCELED ){
                // Change the robot state as Idle
                std::clog << "Resetting robot state to idle after task completion/error reconciliation." << std::endl;
                robot_states->updateRobot(i, -1, fms_msgs::msg::RobotState::MODE_IDLE, robot_states->table[i].pos, "", "");
                return true;
            }
        }

        return false;
    }

    /*  Check whether all tasks are complete
        (If all tasks are complete, idle robots move to their charging stations)  */
    bool MultiAgentStatusMonitor::checkTaskCompletion(std::vector<RobotAction>& action_list){
        
        int count = 0;  // Count QUEUED tasks
        for (int i = 0 ; i < task_states->num_tasks ; i++){
            if (task_states->table[i].state == fms_msgs::msg::TaskState::STATE_QUEUED){
                count++;
            }
        }
        if (count == 0){    // If all tasks are complete
            for (int i = 0 ; i < num_of_robots; i++){
                if (robot_states->table[i].mode == fms_msgs::msg::RobotState::MODE_IDLE){
                    int curr_node = topolomap->getNodeId(robot_states->table[i].pos);
                    if (curr_node == topolomap->nodes_dock[i])   // Already in charging station
                        continue;
                    std::string req_robot_id = robot_states->table[i].RobotID;
                    size_t req_action = fms_msgs::srv::CommandRobot::Request::ACTION_CHARGING;
                    FMS::RobotAction action(0, req_action, req_robot_id, "");
                    action_list.push_back(action);
                }
            }
            if (action_list.size() > 0)
                return true;
        }
        return false;
    }


   /*  Run multi-agent status monitor  
        - Update the robot and task state tables
        - Return the flag for path planning  */
    bool MultiAgentStatusMonitor::runMASM(){

        /*  Update Task & Robot State Tables  */
        bool do_path_planning = false;
        std::fill(updated_robot.begin(), updated_robot.end(), true);
        for (int i = 0 ; i < num_of_robots ; i++){

            Eigen::Vector3d pose, goal;
            std::string TaskID, TaskNode, GoalNode;
            int task = robot_states->table[i].task_id;
            if (task >= 0) {
                TaskID      = task_states->table[task].TaskID;
                TaskNode    = task_states->table[task].curr_task_node;
            }
            pose = robot_states->table[i].pos;
            
            /*  1) Working task operation */
            // 1-1) CTRL_ARRIVE_TASK -> MODE_TASK_PROCESSING
            if (robot_states->table[i].Info == fms_msgs::msg::RobotInfo::CTRL_ARRIVE_TASK &&
                robot_states->table[i].mode == fms_msgs::msg::RobotState::MODE_MOVING_TASK){
                robot_states->updateRobot(i, task, fms_msgs::msg::RobotState::MODE_TASK_PROCESSING, pose, TaskID, TaskNode);
            }

            // 1-2) CTRL_TASK_COMPLETE -> MODE_IDLE
            //   or CTRL_TASK_COMPLETE -> MODE_MOVING
            else if (robot_states->table[i].Info == fms_msgs::msg::RobotInfo::CTRL_TASK_COMPLETE &&
                     robot_states->table[i].mode == fms_msgs::msg::RobotState::MODE_TASK_PROCESSING){
                if (task_states->isLastTaskUnit(task)){ // All task units are complete
                    if (task >= 0){                     // If it is currently processing a task
                        task_states->updateTask(task, -1, fms_msgs::msg::TaskState::STATE_COMPLETED, "");
                    }
                    robot_states->updateRobot(i, -1, fms_msgs::msg::RobotState::MODE_IDLE, pose, "", "");
                }
                else {                                  // Do next task unit
                    task_states->updateNextTaskUnit(task);
                    goal = task_states->table[task].curr_task_loc;
                    TaskNode = task_states->table[task].curr_task_node;
                    robot_states->updateRobot(i, task, fms_msgs::msg::RobotState::MODE_MOVING_TASK, goal, TaskID, TaskNode);
                }
                do_path_planning = true;
                robot_states->setNeedPlanning(i, true);
            }
           
            /*  2) Charging operation */
            // 2-1) CTRL_NEED_CHARGING -> MODE_MOVING_CHARGING
            else if (robot_states->table[i].Info == fms_msgs::msg::RobotInfo::CTRL_NEED_CHARGING){
                if (task >= 0){     // If it is currently processing a task
                    task_states->updateTask(task, -1, fms_msgs::msg::TaskState::STATE_QUEUED, "");
                }
                Eigen::Vector2d loc = topolomap->stations[topolomap->charging_station_index[i]].second;
                goal = Eigen::Vector3d(loc[0], loc[1], 0);             // Target pose (TODO: set yaw value)
                GoalNode = topolomap->stations[topolomap->charging_station_index[i]].first;
                robot_states->updateRobot(i, -1, fms_msgs::msg::RobotState::MODE_MOVING_CHARGING, goal, "", GoalNode);
                do_path_planning = true;
                robot_states->setNeedPlanning(i, true);
            }
            // 2-2) CTRL_ARRIVE_CHARGING -> MODE_CHARGING
            else if (robot_states->table[i].Info == fms_msgs::msg::RobotInfo::CTRL_ARRIVE_CHARGING &&
                     robot_states->table[i].mode == fms_msgs::msg::RobotState::MODE_MOVING_CHARGING){
                robot_states->updateRobot(i, -1, fms_msgs::msg::RobotState::MODE_CHARGING, pose, "", "");
            }
            // 2-3) CTRL_CHARGING_COMPLETE -> MODE_IDLE
            else if (robot_states->table[i].Info == fms_msgs::msg::RobotInfo::CTRL_CHARGING_COMPLETE &&
                     robot_states->table[i].mode == fms_msgs::msg::RobotState::MODE_CHARGING){
                // fms_msgs::RobotState
                robot_states->updateRobot(i, -1, fms_msgs::msg::RobotState::MODE_IDLE, pose, "", "");
            }
            /*  3) Moving operation */
            // 3-1) CTRL_ARRIVE_MOVING -> MODE_IDLE
            else if (robot_states->table[i].Info == fms_msgs::msg::RobotInfo::CTRL_ARRIVE_MOVING &&
                     robot_states->table[i].mode == fms_msgs::msg::RobotState::MODE_MOVING){
                robot_states->updateRobot(i, -1, fms_msgs::msg::RobotState::MODE_IDLE, pose, "", "");
            }
            /*  4) Waiting operation */
            // 4-1) CTRL_ARRIVE_WAITING -> MODE_WAITING
            else if (robot_states->table[i].Info == fms_msgs::msg::RobotInfo::CTRL_ARRIVE_WAITING &&
                     robot_states->table[i].mode == fms_msgs::msg::RobotState::MODE_MOVING_WAITING){
                robot_states->updateRobot(i, -1, fms_msgs::msg::RobotState::MODE_WAITING, pose, "", "");
            }
            // 4-1) CTRL_WAITING_COMPLETE -> MODE_IDLE
            else if (robot_states->table[i].Info == fms_msgs::msg::RobotInfo::CTRL_WAITING_COMPLETE &&
                     robot_states->table[i].mode == fms_msgs::msg::RobotState::MODE_WAITING){
                robot_states->updateRobot(i, -1, fms_msgs::msg::RobotState::MODE_IDLE, pose, "", "");
            }
            /*  5) Error operation */
            else if (robot_states->table[i].Info == fms_msgs::msg::RobotInfo::CTRL_ERROR_OCCURED){
                if (robot_states->table[i].mode == fms_msgs::msg::RobotState::MODE_PAUSED){
                    robot_states->updateRobot(i, -1, fms_msgs::msg::RobotState::MODE_ERROR, pose, "", "");    
                }
                else {
                    int task = robot_states->table[i].task_id;
                    // Save the current robot state in a temporal variable: prev_state
                    robot_states->table[i].prev_state.first  = task;  // If a task is not assigned, just insert -1 value
                    robot_states->table[i].prev_state.second = robot_states->table[i].mode; // Save current mode
                    robot_states->updateRobot(i, task, fms_msgs::msg::RobotState::MODE_ERROR, pose, "", "");
                    if (task >= 0){     // If it is currently processing a task
                        task_states->updateTask(task, i, fms_msgs::msg::TaskState::STATE_PENDING, "");
                    }
                }
                do_path_planning = true;
                robot_states->setNeedPlanning(i, true);
            }
            else {
                updated_robot[i] = false;
            }
        }

        /*  If it requires a path planning, return true  */
        if (do_path_planning){
            std::fill(updated_robot.begin(), updated_robot.end(), true);
            return true;
        }
        
        return do_path_planning;
    }


    void MultiAgentStatusMonitor::updateRobotInfo(std::vector<geometry_msgs::msg::Pose2D> robot_pose_, std::vector<size_t> robot_info_){

        /*  Update poses & progress */
        for (int i = 0 ; i < num_of_robots ; i++){
            robot_states->setPose(i, robot_pose_[i]);       // Update robot poses in state table
            robot_states->table[i].Info = robot_info_[i];   // Update robot information
            robot_states->updateCounter(i);                 // Update progress information
        }
    }

    /*  Run the inserted action commands  */
    bool MultiAgentStatusMonitor::assignRobotActions(std::vector<RobotAction> action_list){

        bool do_path_planning = false;
        std::fill(updated_robot.begin(), updated_robot.end(), false);

        std::vector<bool> is_processed(action_list.size(), false);

        for (int i = 0; i < action_list.size(); i++){
            int robot_i = robot_states->getRobotIndex(action_list[i].robot_id);
            if (robot_i < 0) continue;            
            
            Eigen::Vector3d robot_pose = robot_states->table[robot_i].pos;

            //  1) Pause action
            if (action_list[i].action == fms_msgs::srv::CommandRobot::Request::ACTION_PAUSE){

                if (robot_states->table[robot_i].mode == fms_msgs::msg::RobotState::MODE_PAUSED)
                    continue;

                int task = robot_states->table[robot_i].task_id;
                // Save the current robot state in a temporal variable: prev_state
                robot_states->table[robot_i].prev_state.first  = task;  // If a task is not assigned, just insert -1 value
                robot_states->table[robot_i].prev_state.second = robot_states->table[robot_i].mode; // Save current mode
                robot_states->updateRobot(robot_i, task, fms_msgs::msg::RobotState::MODE_PAUSED, robot_pose, "", "");
                if (task >= 0){     // If it is currently processing a task
                    task_states->updateTask(task, robot_i, fms_msgs::msg::TaskState::STATE_PENDING, ""); // Task state: STATE_PENDING
                }
                do_path_planning = true;
                updated_robot[robot_i] = true;
                robot_states->setNeedPlanning(robot_i, true);
            }
            //  2) Resume action
            else if (action_list[i].action == fms_msgs::srv::CommandRobot::Request::ACTION_RESUME){
                // If the current robot mode is not MODE_PAUSED, disregard resume command.
                if (robot_states->table[robot_i].mode != fms_msgs::msg::RobotState::MODE_PAUSED &&
                    robot_states->table[robot_i].mode != fms_msgs::msg::RobotState::MODE_ERROR)
                    continue;
                int task = robot_states->table[robot_i].prev_state.first;   // previous task (task before pause)
                if (task >= 0){
                    // Resume the previous task
                    size_t mode = robot_states->table[robot_i].prev_state.second;
                    if (mode == fms_msgs::msg::RobotState::MODE_MOVING_TASK || mode == fms_msgs::msg::RobotState::MODE_TASK_PROCESSING){
                        task_states->updateTask(task, robot_i, fms_msgs::msg::TaskState::STATE_ACTIVE, robot_states->table[robot_i].RobotID);
                        robot_states->updateRobot(robot_i, task, mode, task_states->table[task].curr_task_loc, 
                                                  task_states->table[task].TaskID, task_states->table[task].curr_task_node);
                    }
                }
                else {
                    // If a task was not assigned, just set to idle mode
                    robot_states->updateRobot(robot_i, -1, fms_msgs::msg::RobotState::MODE_IDLE, robot_pose, "", "");
                }
                do_path_planning = true;
                updated_robot[robot_i] = true;
                robot_states->setNeedPlanning(robot_i, true);
            }
            else {
                // If current robot mode is MODE_PAUSED or MODE_ERROR
                // disregard the commands (ACTION_IDLE, ACTION_MOVING, ACTION_CHARGING, ACTION_WAITING)
                if (robot_states->table[robot_i].mode == fms_msgs::msg::RobotState::MODE_PAUSED ||
                    robot_states->table[robot_i].mode == fms_msgs::msg::RobotState::MODE_ERROR)
                    continue;

                //  3) Idle action
                if (action_list[i].action == fms_msgs::srv::CommandRobot::Request::ACTION_IDLE){
                    int task_i = robot_states->table[robot_i].task_id;
                    if (task_i >= 0)
                        task_states->updateTask(task_i, -1, fms_msgs::msg::TaskState::STATE_QUEUED, "");
                    robot_states->updateRobot(robot_i, -1, fms_msgs::msg::RobotState::MODE_IDLE, robot_pose, "", "");
                    updated_robot[robot_i] = true;
                    robot_states->setNeedPlanning(robot_i, true);
                }

                //  4) Charging action
                else if (action_list[i].action == fms_msgs::srv::CommandRobot::Request::ACTION_CHARGING){
                    Eigen::Vector2d loc = topolomap->stations[topolomap->charging_station_index[robot_i]].second;
                    Eigen::Vector3d goal = Eigen::Vector3d(loc[0], loc[1], 0);             // Target pose (TODO: set yaw value)
                    std::string node = topolomap->stations[topolomap->charging_station_index[robot_i]].first;
                    int task_i = robot_states->table[robot_i].task_id;
                    if (task_i >= 0)
                        task_states->updateTask(task_i, -1, fms_msgs::msg::TaskState::STATE_QUEUED, "");
                    robot_states->updateRobot(robot_i, -1, fms_msgs::msg::RobotState::MODE_MOVING_CHARGING, goal, "", node);
                    updated_robot[robot_i] = true;
                    do_path_planning = true;
                    robot_states->setNeedPlanning(robot_i, true);
                }
                else {

                    int station_i = topolomap->getStationId(action_list[i].goal_node);   // Target station
                    if (station_i < 0) continue;
                    std::string goal_node = action_list[i].goal_node;
                    Eigen::Vector2d goal_loc = topolomap->stations[station_i].second;
                    Eigen::Vector3d goal_pose(goal_loc[0], goal_loc[1], 0);             // Target pose (TODO: set yaw value)
                    
                    //  5) Moving action
                    if (action_list[i].action == fms_msgs::srv::CommandRobot::Request::ACTION_MOVING){
                        int task_i = robot_states->table[robot_i].task_id;
                        if (task_i >= 0)
                            task_states->updateTask(task_i, -1, fms_msgs::msg::TaskState::STATE_QUEUED, "");
                        robot_states->updateRobot(robot_i, -1, fms_msgs::msg::RobotState::MODE_MOVING, goal_pose, "", goal_node);
                        updated_robot[robot_i] = true;
                        do_path_planning = true;
                        robot_states->setNeedPlanning(robot_i, true);
                    }
                    //  6) Waiting action
                    else if (action_list[i].action == fms_msgs::srv::CommandRobot::Request::ACTION_WAITING){
                        int task_i = robot_states->table[robot_i].task_id;
                        if (task_i >= 0)
                            task_states->updateTask(task_i, -1, fms_msgs::msg::TaskState::STATE_QUEUED, "");
                        robot_states->updateRobot(robot_i, -1, fms_msgs::msg::RobotState::MODE_MOVING_WAITING, goal_pose, "", goal_node);
                        updated_robot[robot_i] = true;
                        do_path_planning = true;
                        robot_states->setNeedPlanning(robot_i, true);
                    }
                    else {
                        continue;
                    }
                }
            }
            is_processed[i] = true;
        }


        for (int i = 0; i < action_list.size(); i++){
            if (is_processed[i]){
                std::clog << "Action [" << action_list[i].action_id << "] was processed successfully." << std::endl;
            }
            else {
                std::clog << "Action [" << action_list[i].action_id << "] was rejected." << std::endl;
            }
        }

        //  If it requires a path planning, return true
        if (do_path_planning){
            std::fill(updated_robot.begin(), updated_robot.end(), true);
            return true;
        }        
        return false;

        
        /// MODIFY HERE
        return true;
    }

}

