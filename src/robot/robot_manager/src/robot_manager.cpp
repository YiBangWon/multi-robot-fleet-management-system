// Robot-manager node that applies route updates and forwards execution state back to the fleet manager.

#include <robot_manager/robot_manager.h>

namespace robot_manager {

    RobotManager::RobotManager(std::string RobotID_, size_t RobotType_, rclcpp::Clock::SharedPtr clock_){
        // Initialize robot manager
        clock = clock_;
        RobotID = RobotID_;
        RobotType = RobotType_;
        task_state.TaskID = "";
        task_state.progress = 0;
        battery_percent = 100;
        robot_mode = fms_msgs::msg::RobotState::MODE_IDLE;   // Initialize to idle state
        ctrl_mode = fms_msgs::msg::RobotInfo::CTRL_IDLE;     // Initialize to idle state
        last_update_time = clock->now();
        battery_update_time = clock->now();
        is_goal_arrival = false;
    }

    // Insert the robot state_msg and update ctrl_mode
    void RobotManager::insert_state_msg(fms_msgs::msg::RobotState state_msg){

        // Update task state
        task_state.TaskID = state_msg.task_id;
        task_state.GoalNode = state_msg.goal_node;
        task_state.target_pose[0] = state_msg.target_pose.x;
        task_state.target_pose[1] = state_msg.target_pose.y;
        task_state.target_pose[2] = state_msg.target_pose.theta;
        task_state.path.clear();
        for (int i = 0; i < state_msg.path.size(); i++){
            Eigen::Vector3d pt(state_msg.path[i].x, state_msg.path[i].y, state_msg.path[i].theta);
            task_state.path.push_back(pt);
        }

        // Update robot controll model
        if (robot_mode != state_msg.mode){                          // If the robot mode is updated
            robot_mode = state_msg.mode;                            // Update new robot mode
            last_update_time = clock->now();                        // Update time

            // --------- Update robot ctrl_mode from robot state ---------
            if (robot_mode == fms_msgs::msg::RobotState::MODE_IDLE){                 // 1) MODE_IDLE -> CTRL_IDLE
                ctrl_mode = fms_msgs::msg::RobotInfo::CTRL_IDLE;
            }
            else if (robot_mode == fms_msgs::msg::RobotState::MODE_MOVING_TASK){     // 2) MODE_MOVING_START -> CTRL_MOVING_START
                ctrl_mode = fms_msgs::msg::RobotInfo::CTRL_MOVING_TASK;
            }
            else if (robot_mode == fms_msgs::msg::RobotState::MODE_TASK_PROCESSING){ // 3) MODE_TASK_LOADING -> CTRL_LOADING
                ctrl_mode = fms_msgs::msg::RobotInfo::CTRL_TASK_PROCESSING;
            }
            else if (robot_mode == fms_msgs::msg::RobotState::MODE_MOVING_CHARGING){ // 6) MODE_MOVING_CHARGING -> CTRL_MOVING_CHARGING
                ctrl_mode = fms_msgs::msg::RobotInfo::CTRL_MOVING_CHARGING;
            }
            else if (robot_mode == fms_msgs::msg::RobotState::MODE_CHARGING){        // 7) MODE_CHARGING -> CTRL_CHARGING
                ctrl_mode = fms_msgs::msg::RobotInfo::CTRL_CHARGING;
            }
            else if (robot_mode == fms_msgs::msg::RobotState::MODE_MOVING){          // 8) MODE_MOVING -> CTRL_MOVING
                ctrl_mode = fms_msgs::msg::RobotInfo::CTRL_MOVING;
            }
            else if (robot_mode == fms_msgs::msg::RobotState::MODE_MOVING_WAITING){  // 9) MODE_MOVING_WAITING -> CTRL_MOVING_WAITING
                ctrl_mode = fms_msgs::msg::RobotInfo::CTRL_MOVING_WAITING;
            }
            else if (robot_mode == fms_msgs::msg::RobotState::MODE_WAITING){         // 10) MODE_WAITING -> CTRL_WAITING
                ctrl_mode = fms_msgs::msg::RobotInfo::CTRL_WAITING;
            }
            else if (robot_mode == fms_msgs::msg::RobotState::MODE_PAUSED){          // 11) MODE_PAUSED -> CTRL_PAUSED
                ctrl_mode = fms_msgs::msg::RobotInfo::CTRL_PAUSED;
            }
            else if (robot_mode == fms_msgs::msg::RobotState::MODE_ERROR){           // 12) MODE_ERROR -> CTRL_ERROR
                ctrl_mode = fms_msgs::msg::RobotInfo::CTRL_ERROR;
            }
            else if (robot_mode == fms_msgs::msg::RobotState::MODE_ISOLATED){        // 12) MODE_ISOLATED -> CTRL_ISOLATED
                ctrl_mode = fms_msgs::msg::RobotInfo::CTRL_ISOLATED;
            }
        }
    }

    void RobotManager::update_state(geometry_msgs::msg::Pose2D pose_)
    {
        /* Update current pose and time */
        curr_pose        = {pose_.x, pose_.y, pose_.theta};
        rclcpp::Time now = clock->now();
        double dt        = std::max((now - last_update_time).seconds(), 1e-3);
        last_update_time = now;

        update_battery_capacity();

        /* Main control mode transitions */
        if (ctrl_mode == fms_msgs::msg::RobotInfo::CTRL_MOVING_TASK) {
            if (check_goal_arrival()) {
                is_goal_arrival = true;
                ctrl_mode       = fms_msgs::msg::RobotInfo::CTRL_ARRIVE_TASK;
                return;
            }
        }
        else if (ctrl_mode == fms_msgs::msg::RobotInfo::CTRL_TASK_PROCESSING) {
            if (check_task_completion()) {
                ctrl_mode = fms_msgs::msg::RobotInfo::CTRL_TASK_COMPLETE;
                return;
            }
        }
        else if (ctrl_mode == fms_msgs::msg::RobotInfo::CTRL_MOVING_CHARGING) {
            if (check_goal_arrival()) {
                is_goal_arrival = true;
                ctrl_mode       = fms_msgs::msg::RobotInfo::CTRL_ARRIVE_CHARGING;
                return;
            }
        }
        else if (ctrl_mode == fms_msgs::msg::RobotInfo::CTRL_CHARGING) {
            if (check_charging_completion()) {
                ctrl_mode = fms_msgs::msg::RobotInfo::CTRL_CHARGING_COMPLETE;
                return;
            }
        }
        else if (ctrl_mode == fms_msgs::msg::RobotInfo::CTRL_MOVING) {
            if (check_goal_arrival()) {
                is_goal_arrival = true;
                ctrl_mode       = fms_msgs::msg::RobotInfo::CTRL_ARRIVE_MOVING;
                return;
            }
        }
        else if (ctrl_mode == fms_msgs::msg::RobotInfo::CTRL_MOVING_WAITING) {
            if (check_goal_arrival()) {
                is_goal_arrival = true;
                ctrl_mode       = fms_msgs::msg::RobotInfo::CTRL_ARRIVE_WAITING;
                return;
            }
        }
        else if (ctrl_mode == fms_msgs::msg::RobotInfo::CTRL_WAITING) {
            if (check_waiting_completion()) {
                ctrl_mode = fms_msgs::msg::RobotInfo::CTRL_WAITING_COMPLETE;
                return;
            }
        }
        else if (ctrl_mode == fms_msgs::msg::RobotInfo::CTRL_PAUSED ||
                ctrl_mode == fms_msgs::msg::RobotInfo::CTRL_ERROR  ||
                ctrl_mode == fms_msgs::msg::RobotInfo::CTRL_ISOLATED) {
            return;  // no further processing
        }

        /* ------------------------------------------------------------------
        * Fallback: treat goal as arrived if robot is near and almost stopped
        * ------------------------------------------------------------------ */
        {
            /* Estimate speed using previous pose */
            static Eigen::Vector3d prev_pose = curr_pose;
            double vx = (curr_pose[0] - prev_pose[0]) / dt;
            double vy = (curr_pose[1] - prev_pose[1]) / dt;
            prev_pose = curr_pose;

            double near_dist     = (curr_pose.head<2>() -
                                    task_state.target_pose.head<2>()).norm();
            double speed         = std::hypot(vx, vy);
            bool   near_and_slow = (near_dist < 1.2 * goal_range) &&
                                (speed     < 0.02);

            bool moving_state =
                (ctrl_mode == fms_msgs::msg::RobotInfo::CTRL_MOVING)          ||
                (ctrl_mode == fms_msgs::msg::RobotInfo::CTRL_MOVING_WAITING)  ||
                (ctrl_mode == fms_msgs::msg::RobotInfo::CTRL_MOVING_TASK)     ||
                (ctrl_mode == fms_msgs::msg::RobotInfo::CTRL_MOVING_CHARGING);

            static rclcpp::Time near_start_time;  // start time of near-and-slow

            if (moving_state && near_and_slow) {
                if (near_start_time.nanoseconds() == 0)
                    near_start_time = now;

                if ((now - near_start_time).seconds() > 2.0) {
                    is_goal_arrival = true;

                    switch (ctrl_mode)
                    {
                        case fms_msgs::msg::RobotInfo::CTRL_MOVING_TASK:
                            ctrl_mode = fms_msgs::msg::RobotInfo::CTRL_ARRIVE_TASK; break;
                        case fms_msgs::msg::RobotInfo::CTRL_MOVING_CHARGING:
                            ctrl_mode = fms_msgs::msg::RobotInfo::CTRL_ARRIVE_CHARGING; break;
                        case fms_msgs::msg::RobotInfo::CTRL_MOVING_WAITING:
                            ctrl_mode = fms_msgs::msg::RobotInfo::CTRL_ARRIVE_WAITING; break;
                        default:
                            ctrl_mode = fms_msgs::msg::RobotInfo::CTRL_ARRIVE_MOVING; break;
                    }
                    return;
                }
            } else {
                near_start_time = rclcpp::Time(0, 0);
            }
        }
        /* End of fallback logic */

        /* Check system error */
        if (check_error()) {
            ctrl_mode = fms_msgs::msg::RobotInfo::CTRL_ERROR_OCCURED;
            return;
        }

        /* Battery check (only in IDLE or MOVING_TASK) */
        if (ctrl_mode == fms_msgs::msg::RobotInfo::CTRL_IDLE ||
            ctrl_mode == fms_msgs::msg::RobotInfo::CTRL_MOVING_TASK) {
            if (check_battery_charging()) {
                ctrl_mode = fms_msgs::msg::RobotInfo::CTRL_NEED_CHARGING;
            }
        }
    }



    // Check goal arrival
    bool RobotManager::check_goal_arrival(){
        double dist = (curr_pose.head(2) - task_state.target_pose.head(2)).norm();
        if (dist < goal_range)  return true;
        else                    return false;
    }

    // Check loading completion: simply check processing time
    bool RobotManager::check_task_completion(){
        double processing_time = (clock->now() - last_update_time).seconds();
        if (processing_time > thr_task_time){
            return true;
        }
        return false;
    }

    // Update virtual battery capacity:
    //    - Normal: decrease battery capacity according to thr_battery_dec_time
    //    - Charging station: increment battery capacity according to thr_battery_inc_time
    void RobotManager::update_battery_capacity(){
        double update_time = (clock->now() - battery_update_time).seconds();
        if (ctrl_mode == fms_msgs::msg::RobotInfo::CTRL_CHARGING){
            if (update_time > thr_battery_inc_time){
                battery_update_time = clock->now();
                battery_percent = std::min(100, (int)(battery_percent + 1));
            }
        }
        else {
            if (update_time > thr_battery_dec_time){
                battery_update_time = clock->now();
                battery_percent = std::max(0, (int)(battery_percent - 1));
            }
        }
    }

    // Check battery capacity (need to charging?)
    bool RobotManager::check_battery_charging(){
        if (battery_percent < thr_min_battery_percent){
            if (ctrl_mode == fms_msgs::msg::RobotInfo::CTRL_IDLE ||
                ctrl_mode == fms_msgs::msg::RobotInfo::CTRL_MOVING_TASK){
                return true;
            }
        }
        return false;
    }

    // Check charging completion based on thr_max_battery_percent
    bool RobotManager::check_charging_completion(){
        if (battery_percent > thr_max_battery_percent){
            return true;
        }
        return false;
    }

    // Check waiting completion based on thr_waiting_time
    bool RobotManager::check_waiting_completion(){
        double processing_time = (clock->now() - last_update_time).seconds();
        if (processing_time > thr_waiting_time){
            return true;
        }
        return false;
    }

    // Check waiting completion based on thr_waiting_time
    bool RobotManager::check_error(){
        // srand(time(0));

        // if (rand()%1000 == 0){
        //     return true;
        // }
        return false;
    }
}
