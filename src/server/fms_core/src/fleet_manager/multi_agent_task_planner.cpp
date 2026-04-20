// Task allocation module that assigns queued tasks and updates task timing estimates.

#include <fleet_manager/multi_agent_task_planner.h>
#define MAX_DIST 99999

namespace FMS
{
    /*  Initialize the MATP module  */
    void MultiAgentTaskPlanner::initMATP(TaskStates* task_states_, RobotStates* robot_states_, TopologicalMap* topolomap_,
                                         PathPlanner* path_planner_, SolverMap* solver_map_){
        task_states  = task_states_;                // Task state table
        robot_states = robot_states_;               // Robot state table
        topolomap = topolomap_;                     // Topological map
        num_of_robots = robot_states->num_robots;   // Number of robots
        // Single agent path planner in MATP (for distance measure)
        path_planner = path_planner_;               // Single agent path planner (A* search)
        solver_map = solver_map_;                   // Solver map
        setTaskNodeIDs();
    }

    /*  Set IDs of task nodes  */
    void MultiAgentTaskPlanner::setTaskNodeIDs(){
        for (int i = 0; i < task_states->num_tasks; i++){
            for (int j = 0; j < task_states->table[i].task_units.size(); j++){
                Eigen::Vector2d loc = task_states->table[i].task_units[j].location.head(2);
                int id = topolomap->getNodeId(loc);
                task_states->table[i].task_units[j].node_id = id;
            }
        }
    }

    /*  Get the distance of a path from start to goal  */
    double MultiAgentTaskPlanner::getDistance(int start, int goal){
        // Shortest path distance is equal to the heuristic value of the path planner
        if (solver_map->heuristics.find(goal) == solver_map->heuristics.end()){
            // If the heuristic value is not computed yet, update the heuristic value
            solver_map->heuristics[goal] = path_planner->getHeuristics(*solver_map, goal);
        }
        double dist = solver_map->heuristics[goal][start]; // Get the heuristic value
		return dist;
	}

    /* Update remaining & processing time for each task */
    void MultiAgentTaskPlanner::updateTimes(){

        for (int robot_i = 0; robot_i < robot_states->num_robots; robot_i++){
            if (robot_states->table[robot_i].mode == fms_msgs::msg::RobotState::MODE_MOVING_TASK ||
                robot_states->table[robot_i].mode == fms_msgs::msg::RobotState::MODE_TASK_PROCESSING){

                int task_i = robot_states->table[robot_i].task_id;
                // task_states->table[task_i].processing_time = rclcpp::Time((rclcpp::Time::now() - task_states->table[task_i].start_time).sec);

                Eigen::Vector3d pose = robot_states->table[robot_i].pos;
                int curr_node_id = topolomap->getNodeId(pose);
                std::vector<TaskUnit> task_units = task_states->table[task_i].task_units;
                double estimated_time = 0;
                int index = task_states->table[task_i].curr_task_index;
                for (int j = index; j < task_units.size(); j++){
                    int task_node_id = task_units[j].node_id;
                    double dist = getDistance(curr_node_id, task_node_id);
                    curr_node_id = task_node_id;
                    estimated_time += dist*sec_per_node + sec_per_task;
                }
                task_states->table[task_i].remaining_time = rclcpp::Time(estimated_time);
            }
        }
    }

    /*  Run multi-agent task planning   
        - Update the robot and task state tables according to the assigned tasks
        - Return the flag for path planning  */
    bool MultiAgentTaskPlanner::runMATP(){
        bool updated;
        if (is_greedy_assignment)
            updated = GreedyAssignment();
        else
            updated = SequentialAssignment();
        return updated;
    }

     /*  Update assigned tasks from update topological map */
    bool MultiAgentTaskPlanner::updateMATP(){

        // 1) Find disabled robots by checking the accessibility to their duck nodes
        int sel_robot = 0;
        std::vector<int> duck_nodes(num_of_robots);
        for (int robot_i = 0; robot_i < num_of_robots; robot_i++){
            // Check this robot can move to its duck node
            Eigen::Vector3d pose = robot_states->table[robot_i].pos;
            int robot_node = topolomap->getNodeId(pose);
            int duck_node  = topolomap->nodes_dock[robot_i];
            duck_nodes[robot_i] = duck_node;
            double dist = getDistance(robot_node, duck_node);
            if (dist >= WEIGHT_MAX){    // This robot cannot move to the duck node
                int task_i = robot_states->table[robot_i].task_id;
                if (task_i >= 0)
                    task_states->updateTask(task_i, -1, fms_msgs::msg::TaskState::STATE_QUEUED, "");
                robot_states->updateRobot(robot_i, -1, fms_msgs::msg::RobotState::MODE_ISOLATED, pose, "", ""); // Set the mode to ERROR
                if (sel_robot == robot_i)
                    sel_robot++;
            }
            else if (robot_states->table[robot_i].mode == fms_msgs::msg::RobotState::MODE_ISOLATED){
                robot_states->updateRobot(robot_i, -1, fms_msgs::msg::RobotState::MODE_IDLE, pose, "", ""); // Set the mode to IDLE
            }
        }
        if (sel_robot >= num_of_robots) return false;

        // 2) Find disabled tasks by checking the accessibility to duck nodes
        for (int task_i = 0; task_i < task_states->num_tasks; task_i++){

            if (task_states->table[task_i].state == fms_msgs::msg::TaskState::STATE_CANCELED  ||
                task_states->table[task_i].state == fms_msgs::msg::TaskState::STATE_COMPLETED ||
                task_states->table[task_i].state == fms_msgs::msg::TaskState::STATE_PENDING)
                continue;

            bool found = false;
            std::vector<TaskUnit> task_units = task_states->table[task_i].task_units;
            for (int k = 0; k < task_units.size(); k++){
                double dist = getDistance(task_units[k].node_id, duck_nodes[sel_robot]);
                if (dist >= WEIGHT_MAX){    // This robot cannot move to this task node
                    found = true;
                    break;
                }
            }

            if (found){
                if (task_states->table[task_i].state == fms_msgs::msg::TaskState::STATE_ACTIVE){
                    int robot_i = task_states->table[task_i].robot_id;
                    Eigen::Vector3d pose = robot_states->table[robot_i].pos;
                    robot_states->updateRobot(robot_i, -1, fms_msgs::msg::RobotState::MODE_IDLE, pose, "", ""); // Set the mode to IDLE
                    task_states->updateTask(task_i, -1, fms_msgs::msg::TaskState::STATE_FAILED, "");
                }
                else if (task_states->table[task_i].state == fms_msgs::msg::TaskState::STATE_QUEUED){
                    task_states->updateTask(task_i, -1, fms_msgs::msg::TaskState::STATE_FAILED, "");
                }
            }
            else {
                if (task_states->table[task_i].state == fms_msgs::msg::TaskState::STATE_FAILED){
                    task_states->updateTask(task_i, -1, fms_msgs::msg::TaskState::STATE_QUEUED, "");
                }
            }
        }

        return true;
    }
    
     /*  Sequential task assignment: simply assign tasks sequentially  */
    bool MultiAgentTaskPlanner::SequentialAssignment(){

        int num_assigned_tasks = 10;
        bool is_updated = false;

        for (int iter = 0; iter < num_assigned_tasks; iter++){
            // 1. Find a pair of task (STATE_QUEUED) and robot (MODE_IDLE)
            int select_task  = -1;  // Selected task
            int select_robot = -1;  // Selected robot
            // 1.1. Get QUEUED task
            for (int i = 0 ; i < task_states->num_tasks ; i++){
                if (task_states->table[i].state == fms_msgs::msg::TaskState::STATE_QUEUED){
                    select_task = i;
                    break;
                }
            }
            // 1.2. Get IDLE robot
            for (int i = 0 ; i < num_of_robots ; i++){
                if (robot_states->table[i].mode == fms_msgs::msg::RobotState::MODE_IDLE){
                    select_robot = i;
                    break;
                }
            }
            // 2. If there is a matched pair, update their tables
            if (select_task >= 0 && select_robot >= 0) {
                // 2.1. Assign a new task to a idle robot!
                task_states->updateTask(select_task, select_robot, fms_msgs::msg::TaskState::STATE_ACTIVE,         // Update task state table
                                        robot_states->table[select_robot].RobotID);
                robot_states->updateRobot(select_robot, select_task, fms_msgs::msg::RobotState::MODE_MOVING_TASK,  // Update robot state table
                                        task_states->table[select_task].curr_task_loc, task_states->table[select_task].TaskID, 
                                        task_states->table[select_task].curr_task_node);
                // 2.2. Return the success of task assignment
                robot_states->setNeedPlanning(select_robot, true);
                is_updated = true;
            }
            else {
                break;
            }
        }
        return is_updated;
    }

    /*  Greedy task assignment: assign the task with the minimum path distance  */
    bool MultiAgentTaskPlanner::GreedyAssignment(){

        bool is_updated = false;
        for (int iter = 0; iter < num_assigned_tasks; iter++){
            std::vector<int> task_set;              // Candidate set
            std::vector<int> task_s_node_set;       // Candidate set (start node)
            int select_robot = -1;                  // Selected robot
            int robot_node = -1;

            // 1. Get IDLE robot
            for (int i = 0; i < num_of_robots; i++){
                if (robot_states->table[i].mode == fms_msgs::msg::RobotState::MODE_IDLE){
                    select_robot = i;
                    robot_node = topolomap->getNodeId(robot_states->table[i].pos);
                    break;
                }
            }
            if (select_robot < 0) { break; }

            // 2. Get set of QUEUED tasks
            bool found = false;
            for (int priority = 0; priority <= 5; priority++){
                for (int i = 0; i < task_states->num_tasks; i++){
                    if (task_states->table[i].state == fms_msgs::msg::TaskState::STATE_QUEUED &&  // If its state is STATE_QUEUED
                        task_states->table[i].priority == priority){                         // and its priority is the current priority, 
                        task_set.push_back(i);                                               // insert it into the candidate set
                        task_s_node_set.push_back(topolomap->getNodeId(task_states->table[i].task_units[0].location));
                    }
                }
                if (task_set.size() > 0){
                    found = true;
                    break;
                }
            }
            if (!found) { break; }

            // 3. Select the best task using pure greedy assignment
            int select_task = -1;
            double min_dist = MAX_DIST;
            for (int i = 0; i < task_set.size(); i++){
                double dist = getDistance(robot_node, task_s_node_set[i]);  // Compute distance b/w current and target locations
                if (dist < min_dist){                                       // Set the minimum distance task
                    min_dist = dist;
                    select_task = task_set[i];
                }
            }
            if (select_task < 0)  { break; }
            // 4. Assign a new task to a idle robot!
            task_states->updateTask(select_task, select_robot, fms_msgs::msg::TaskState::STATE_ACTIVE,         // Update task state table
                                    robot_states->table[select_robot].RobotID);
            robot_states->updateRobot(select_robot, select_task, fms_msgs::msg::RobotState::MODE_MOVING_TASK,       // Update robot state table
                                    task_states->table[select_task].curr_task_loc, task_states->table[select_task].TaskID,
                                    task_states->table[select_task].curr_task_node);
            robot_states->setNeedPlanning(select_robot, true);
            is_updated = true;
        }
        return is_updated;
    }
}