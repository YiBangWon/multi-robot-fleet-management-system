// Multi-agent route planning module built on ECBS-style search and topology-aware heuristics.

#include <iostream>

#include <fleet_manager/multi_agent_route_planner.h>
// #include <fleet_manager/utils.h>
#define PI 3.14159

namespace FMS
{
/*
    Ref_RHCR: Li et al. 2021: Lifelong Multi-Agent Path Finding in Large-Scale Warehouses, AAAI 2021
    Ref_ECBS: Barer et al. 2014: Suboptimal Variants of the Conflict-Based Search Algorithm for the Multi-Agent Pathfinding Problem, SoCS 2014
*/

/* Initialize the MARP module */
void MultiAgentRoutePlanner::initMARP(TaskStates* task_states_, RobotStates* robot_states_, TopologicalMap* topolomap_){
    task_states  = task_states_;                    // Task state table
    robot_states = robot_states_;                   // Robot state table
    topolomap = topolomap_;                         // Toplogical map
    num_of_robots = robot_states->num_robots;       // Number of robots
    remove_stations = topolomap->remove_stations;    // True: remove station nodes
    setSolverMap();                                 // Initialize solver map from topological map
    initSolver();                                   // Initialize solver
}

/* Set the parameter of MARP module */
void MultiAgentRoutePlanner::setParams(int window_, int window_extend_, int time_limit_, double suboptimal_bound_){
    window              = window_;              // Window: bounded time horizon
    window_extend       = window_extend_;       // Extended window for single agent planner (A* Search)
    time_limit          = time_limit_;          // Time limit for solver
    suboptimal_bound    = suboptimal_bound_;    // Suboptimal bound for ECBS
}

/* Initialize the solver's map from topological map */
void MultiAgentRoutePlanner::setSolverMap(){

    // Set yaw value for each direction
    yaws.resize(4);
    yaws[0] = std::atan2(0, 1);     // Right
    yaws[1] = std::atan2(-1, 0);    // Down
    yaws[2] = std::atan2(0,-1);     // Left
    yaws[3] = std::atan2(1, 0);     // Up

    // 1) Set node information
    solver_map.num_nodes = topolomap->num_nodes;  // Insert the node into the solver's map (They are same)
    for (int i = 0; i < topolomap->num_nodes; i++){
        // For each node, set its attributes: location, neighbor, reverse_neighbor, weight, type.
        // The tables of neighbor and weight represent the node's connections in 4-directions, which represent the graph map
        Eigen::Vector2d location = topolomap->nodes[i].loc;  // Node location
        std::vector<int> neighbor(5, -1);                        // Node neighbors
        std::vector<int> rneighbor(5, -1);                       // Node reverse neighbors (required for reverse tracking of neighbors)
        std::vector<double> weight(5, WEIGHT_MAX);               // Node weights   (In this program, we use the unit weight 1)
        // These are Nx5 tables representing the neighbor information on graph map
        // Column1: right, Column2: down, Column3: left, Column4: up, Column5: stay
        neighbor[4]  = i;
        rneighbor[4] = i;
        weight[4]    = weight_wait;   // Unit weight
        // Set nodes
        solver_map.locations.push_back(location);
        solver_map.neighbors.push_back(neighbor);
        solver_map.rneighbors.push_back(rneighbor);
        solver_map.weights.push_back(weight);
    }

    // 2) Set the connection information
    for (int i = 0 ; i < topolomap->num_lanes ; i++){

        bool bidirect = topolomap->lanes[i].bidirect;            // Ture: bi-directionaly lane
        int i_head = topolomap->lanes[i].entry;                  // Entry node id in topological map
        int i_tail = topolomap->lanes[i].exit;                   // Exit node id in topological map
        Eigen::Vector2d head = topolomap->nodes[i_head].loc;     // Entry node location
        Eigen::Vector2d tail = topolomap->nodes[i_tail].loc;     // Exit node location
        // Get heading direction (yaw)
        Eigen::Vector2d vec = tail - head;
        double dist = weight_move; // Unit weight
        double yaw  = std::atan2(vec[1], vec[0]);

        std::vector<bool> check(yaws.size(), true);
        int dir = getMinDir(yaw, yaws, check);              // Discrete diection (0: Right, 1: Down, 2: Left, 3: Up)
        int dir_inv = (dir + 2) % 4;                        // Inverse direction can be computed simply
        if (solver_map.neighbors[i_head][dir] >= 0){        // If the direction of this lane is already assigned (i.e. diagonal line)
            std::vector<bool> check(yaws.size(), true);
            for (int j = 0; j < 4; j++){
                int j_inv = (j + 2) % 4;
                if (solver_map.neighbors[i_head][j] >= 0 || solver_map.neighbors[i_tail][j_inv] >= 0)
                    check[j] = false;
            }
            dir = getMinDir(yaw, yaws, check);                  // Compute other closet direction
        }
        solver_map.neighbors[i_head][dir]      = i_tail;        // Connect the head to tail
        solver_map.rneighbors[i_tail][dir]     = i_head;        // Set the reverse table
        solver_map.weights[i_head][dir]        = dist;          // Set the weight of the connection
        if (bidirect){
            solver_map.neighbors[i_tail][dir_inv]  = i_head;    // Add inverse connection for the bidirected edge
            solver_map.rneighbors[i_head][dir_inv] = i_tail;    // Add inverse connection for the bidirected edge
            solver_map.weights[i_tail][dir_inv]    = dist;      // Add inverse connection for the bidirected edge
        }
    }
    solver_map.rotation = true;

    // Set robot stations (dummy goal) for dummy path planning
    setStations();
}


/* Reset the solver's map from updated topological map */
void MultiAgentRoutePlanner::updateSolverMap(){

    // 1) Set node information
    solver_map.num_nodes = topolomap->num_nodes;  // Insert the node into the solver's map (They are same)
    for (int i = 0; i < topolomap->num_nodes; i++){
        std::vector<int> neighbor(5, -1);
        std::vector<int> rneighbor(5, -1);
        std::vector<double> weight(5, WEIGHT_MAX);
        neighbor[4]  = i;
        rneighbor[4] = i;
        weight[4]    = weight_wait;   // Unit weight
        solver_map.neighbors[i].clear();
        solver_map.rneighbors[i].clear();
        solver_map.weights[i].clear();
        solver_map.neighbors[i]  = neighbor;
        solver_map.rneighbors[i] = rneighbor;
        solver_map.weights[i]    = weight;
    }

    // 2) Set the connection information
    for (int i = 0 ; i < topolomap->num_lanes ; i++){

        if (topolomap->lanes[i].disabled) continue;

        bool bidirect = topolomap->lanes[i].bidirect;            // Ture: bi-directionaly lane
        int i_head = topolomap->lanes[i].entry;                  // Entry node id in topological map
        int i_tail = topolomap->lanes[i].exit;                   // Exit node id in topological map
        Eigen::Vector2d head = topolomap->nodes[i_head].loc;     // Entry node location
        Eigen::Vector2d tail = topolomap->nodes[i_tail].loc;     // Exit node location
        // Get heading direction (yaw)
        Eigen::Vector2d vec = tail - head;
        double dist = weight_move; // Unit weight
        double yaw  = std::atan2(vec[1], vec[0]);

        std::vector<bool> check(yaws.size(), true);
        int dir = getMinDir(yaw, yaws, check);              // Discrete diection (0: Right, 1: Down, 2: Left, 3: Up)
        int dir_inv = (dir + 2) % 4;                        // Inverse direction can be computed simply
        if (solver_map.neighbors[i_head][dir] >= 0){        // If the direction of this lane is already assigned (i.e. diagonal line)
            std::vector<bool> check(yaws.size(), true);
            for (int j = 0; j < 4; j++){
                int j_inv = (j + 2) % 4;
                if (solver_map.neighbors[i_head][j] >= 0 || solver_map.neighbors[i_tail][j_inv] >= 0)
                    check[j] = false;
            }
            dir = getMinDir(yaw, yaws, check);                  // Compute other closet direction
        }
        solver_map.neighbors[i_head][dir]      = i_tail;        // Connect the head to tail
        solver_map.rneighbors[i_tail][dir]     = i_head;        // Set the reverse table
        solver_map.weights[i_head][dir]        = dist;          // Set the weight of the connection
        if (bidirect){
            solver_map.neighbors[i_tail][dir_inv]  = i_head;    // Add inverse connection for the bidirected edge
            solver_map.rneighbors[i_head][dir_inv] = i_tail;    // Add inverse connection for the bidirected edge
            solver_map.weights[i_tail][dir_inv]    = dist;      // Add inverse connection for the bidirected edge
        }
    }
    solver_map.heuristics.clear();
    setStations();
}

/*  Initialize the MAPF solver  */
void MultiAgentRoutePlanner::initSolver(){
    path_planner = new PathPlanner();
    path_planner->num_nodes = solver_map.num_nodes;
    path_planner->window   = window_extend;
    solver = new ECBS(solver_map, *path_planner);
    solver->suboptimal_bound = suboptimal_bound;
    solver->window = window;
}

/* Run MAPF solver and get the final routes (Run RHCR algorithm with ECBS solver) */
bool MultiAgentRoutePlanner::runMAPFSolver(){

    // 1) Set start states and goal states with dummy goal
    std::vector<State> starts(start_loc.size());                    // Start state for solver
    std::vector<std::vector<std::pair<int, int>>> goals(start_loc.size());    // Goal state for solver
    for (int k = 0; k < start_loc.size(); k++){
		starts[k] = State(start_loc[k].first, 0, start_loc[k].second);
        goals[k].emplace_back(goal_loc[k].first, goal_loc[k].second);
        double h = solver_map.heuristics[goal_loc[k].first][start_loc[k].first];
        goals[k].emplace_back(dummy_goal_loc[k].first, dummy_goal_loc[k].second); // Insert dummy goals (isolated locations)
    }
    // 2) Run MAPF solver
    // rclcpp::Time start_time = ros::Time::now();
    path_planner->init_cts.clear();                                 // Clear initial conflicts
    bool solution_found = solver->Run(starts, goals, time_limit);   // Run MAPF solver
    // std::cout << "Planning Time: " << (ros::Time::now() - start_time).toSec() << " sec" << std::endl;
    // 3) If solution is found, get the final paths. (Else return false)
    if (solution_found){
        solution.clear();
        solution = solver->solution;    // Update the solution
        setRoutesFromSolution();        // Set the variables of routes from the computed MAPF solution
        return true;
    }
    else {
        std::cerr << "No MAPF solution found for the current planning window." << std::endl;
        return false;
    }
    return true;
}

/* Update the routes from the computed MAPF solution */
void MultiAgentRoutePlanner::setRoutesFromSolution(){

    routes.clear();
    routes.resize(num_of_robots);
    for (int i = 0; i < solution.size(); i++){

        int n_count = 0;                                          // Number of nodes in the route

        if (robot_states->table[i].mode == fms_msgs::msg::RobotState::MODE_ISOLATED){
            NodeRoute node;
            Eigen::Vector2d loc = robot_states->table[i].loc;
            int id = topolomap->getNodeId(loc);
            node.pose   = Eigen::Vector3d(loc[0], loc[1], 0);
            node.loc_id = id;
            node.is_dummy = false;
            node.is_waypoint = topolomap->nodes[id].is_waypoint;
            // node.is_waypoint = true;
            routes[i].push_back(node);
            continue;
        }

        bool reach_goal = false;
        for (int t = 0; t < solution[i].size(); t++){
            if (t > solver->window){ break; }                           // Only set the nodes within bounded horizon (window)
            int id = solution[i][t].loc;                                // Location index
            double yaw = yaws[solution[i][t].dir];                      // Get the 4-directed yaw value
            Eigen::Vector2d loc = topolomap->nodes[id].loc;              // Get the node location from the topological map
            NodeRoute node;                                             // Route node
            node.pose   = Eigen::Vector3d(loc[0], loc[1], yaw);         // Waypoint pose is composed of [x, y, yaw]
            node.loc_id = id;
            if(fms_mode == 1){
                node.is_waypoint = topolomap->nodes[id].is_waypoint;
            }
            else if(fms_mode == 0){
                node.is_waypoint = true;                                           // Set the node index
            }
            else if(fms_mode == 2){
                if(id == goal_loc[i].first){
                    node.is_waypoint = true;
                }
                else{
                    node.is_waypoint = false;
                }
            }
            else{
                std::cerr << "Unsupported fms_mode value: " << fms_mode << std::endl;
                node.is_waypoint = false;
            }


            // if(t == 0) node.is_waypoint = true;


            if (reach_goal){ node.is_dummy = true;  }                   // If it reaches the first goal, this is a dummy node
            else           {
                node.is_dummy = false;
                n_count++;
            }
            routes[i].push_back(node);
            // if(node.is_waypoint) routes[i].push_back(node);
            // else if(!routes[i].empty()) routes[i].push_back(routes[i].back());
            // else routes[i].push_back(node);
            if (id == goal_loc[i].first){
                reach_goal = true;             // Path index at the first goal
                node.is_waypoint = true;
            }
        }

        if(solution[i].empty() || n_count <= 1){
            for(int t = 0; t < 3; t++){
                NodeRoute node;
                Eigen::Vector2d loc = robot_states->table[i].loc;
                int id = topolomap->getNodeId(loc);
                node.pose   = Eigen::Vector3d(loc[0], loc[1], 0);
                node.loc_id = id;
                node.is_dummy = false;
                node.is_waypoint = true;
                routes[i].push_back(node);
            }
        }
    }
}

/* Set the start and goal locations from the robot state tables */
void MultiAgentRoutePlanner::setStartsAndGoals(std::vector<std::pair<bool, Eigen::Vector3d>> start_nodes){
    // Reset the starts and goals
    start_poses.clear();   // Start pose
    goal_poses.clear();    // Goal pose
    start_loc.clear();     // Start location (with direction)
    goal_loc.clear();      // Goal location (with direction)

    // Set the starts and goals from the robot state table
    for (int i = 0; i < num_of_robots; i++) {
        Eigen::Vector3d start_pose = robot_states->table[i].pos;
        Eigen::Vector3d goal_pose  = robot_states->table[i].target;
        if (start_nodes[i].first){                  // True: set the last node in an initial route table as a start
            start_pose = start_nodes[i].second;
        }
        // If robot mode is MODE_IDLE, MODE_TASK_LOADING, MODE_TASK_UNLOADING, MODE_CHARGING, MODE_WAITING, or MODE_PAUSED,
        // just assign the target to the current position
        if (robot_states->table[i].mode == fms_msgs::msg::RobotState::MODE_IDLE            ||
            robot_states->table[i].mode == fms_msgs::msg::RobotState::MODE_TASK_PROCESSING ||
            robot_states->table[i].mode == fms_msgs::msg::RobotState::MODE_CHARGING        ||
            robot_states->table[i].mode == fms_msgs::msg::RobotState::MODE_WAITING         ||
            robot_states->table[i].mode == fms_msgs::msg::RobotState::MODE_PAUSED          ||
            robot_states->table[i].mode == fms_msgs::msg::RobotState::MODE_ERROR){
            goal_pose = start_pose;
        }
        else if (robot_states->table[i].mode == fms_msgs::msg::RobotState::MODE_ISOLATED){
            Eigen::Vector2d loc = topolomap->nodes[dummy_goal_loc[i].first].loc;
            start_pose = Eigen::Vector3d(loc[0], loc[1], 0);
            goal_pose  = Eigen::Vector3d(loc[0], loc[1], 0);
        }
        start_poses.push_back(start_pose);
        goal_poses.push_back(goal_pose);
        int start_id = topolomap->getNodeId(start_pose);    // Get the node id from the location information
        int goal_id  = topolomap->getNodeId(goal_pose);     // Get the node id from the location information
        std::pair<int, int> s_(start_id, getDirection(start_pose[2]));  // <node_index, direction_index>
        std::pair<int, int> g_(goal_id, getDirection(goal_pose[2]));    // <node_index, direction_index>
        start_loc.push_back(s_);
        goal_loc.push_back(g_);
    }
}

/* Set the start and goal locations from the robot state tables */
void MultiAgentRoutePlanner::setStartsAndGoals(std::vector<Eigen::Vector3d> curr_poses, std::vector<std::pair<bool, Eigen::Vector3d>> start_nodes){
    // Reset the starts and goals
    start_poses.clear();   // Start pose
    goal_poses.clear();    // Goal pose
    start_loc.clear();     // Start location (with direction)
    goal_loc.clear();      // Goal location (with direction)

    // Set the starts and goals from the robot state table
    for (int i = 0; i < num_of_robots; i++) {
        Eigen::Vector3d start_pose = curr_poses[i];
        Eigen::Vector3d goal_pose  = robot_states->table[i].target;
        if (start_nodes[i].first){                  // True: set the last node in an initial route table as a start
            start_pose = start_nodes[i].second;
        }
        // If robot mode is MODE_IDLE, MODE_TASK_LOADING, MODE_TASK_UNLOADING, MODE_CHARGING, MODE_WAITING, or MODE_PAUSED,
        // just assign the target to the current position
        if (robot_states->table[i].mode == fms_msgs::msg::RobotState::MODE_IDLE            ||
            robot_states->table[i].mode == fms_msgs::msg::RobotState::MODE_TASK_PROCESSING ||
            robot_states->table[i].mode == fms_msgs::msg::RobotState::MODE_CHARGING        ||
            robot_states->table[i].mode == fms_msgs::msg::RobotState::MODE_WAITING         ||
            robot_states->table[i].mode == fms_msgs::msg::RobotState::MODE_PAUSED          ||
            robot_states->table[i].mode == fms_msgs::msg::RobotState::MODE_ERROR){
            goal_pose = start_pose;
        }
        else if (robot_states->table[i].mode == fms_msgs::msg::RobotState::MODE_ISOLATED){
            Eigen::Vector2d loc = topolomap->nodes[dummy_goal_loc[i].first].loc;
            start_pose = Eigen::Vector3d(loc[0], loc[1], 0);
            goal_pose  = Eigen::Vector3d(loc[0], loc[1], 0);
        }
        start_poses.push_back(start_pose);
        goal_poses.push_back(goal_pose);
        int start_id = topolomap->getNodeId(start_pose);    // Get the node id from the location information
        int goal_id  = topolomap->getNodeId(goal_pose);     // Get the node id from the location information
        std::pair<int, int> s_(start_id, getDirection(start_pose[2]));  // <node_index, direction_index>
        std::pair<int, int> g_(goal_id, getDirection(goal_pose[2]));    // <node_index, direction_index>
        start_loc.push_back(s_);
        goal_loc.push_back(g_);
    }
}


/* Update heuristic information of the start and goal nodes for A* path planning */
void MultiAgentRoutePlanner::updateHeuristics(){
    for (int i = 0; i < num_of_robots; i++) {
        int goal_id = goal_loc[i].first;
        int start_id = start_loc[i].first;
    	if (solver_map.heuristics.find(goal_id) == solver_map.heuristics.end()){
            // If the heuristics for the goal node is not computed yet,
            // compute and update the heuristics for A* search
            solver_map.heuristics[goal_id] = path_planner->getHeuristics(solver_map, goal_id);
        }
        /*
        if (solver_map.heuristics.find(start_id) == solver_map.heuristics.end()){
            solver_map.heuristics[start_id] = path_planner->getHeuristics(solver_map, start_id);
        }
        */
    }
}

/* Correct an overlapping start to a neighbor location for MAPF solver */
void MultiAgentRoutePlanner::correctOverlappingStarts(){
    std::vector<int> start_ids(num_of_robots);
    for (int i = 0; i < num_of_robots; i++){
        start_ids[i] = start_loc[i].first;
    }

    std::vector<bool> is_overlapped(num_of_robots, false);
    std::vector<std::vector<int>> overlapped_sets;
    for (int i = 0; i < num_of_robots; i++) {
        if (is_overlapped[i]) continue;
        std::vector<int> overlapped_set;
        overlapped_set.push_back(i);
        for (int j = 0; j < num_of_robots; j++){
            if (i == j) continue;
            if (start_ids[i] == start_ids[j]){
                is_overlapped[j] = true;
                overlapped_set.push_back(j);
            }
        }
        if (overlapped_set.size() > 1){
            overlapped_sets.push_back(overlapped_set);
        }
    }

    if (overlapped_sets.size() > 0){

        for (int i = 0; i < overlapped_sets.size(); i++) {
            for (int j = 0; j < overlapped_sets[i].size(); j++) {
                int index = overlapped_sets[i][j];
            }
        }
        for (int i = 0; i < overlapped_sets.size(); i++) {
            int num_correct = 0;
            for (int j = 0; j < overlapped_sets[i].size(); j++) {
                int robot = overlapped_sets[i][j];
                int s_id = start_ids[robot];
                int g_id = goal_loc[robot].first;
                std::vector<int> nbrs =solver_map.neighbors[s_id];
                double min_dist = 9999;
                int min_nbr_id = 0;
                for (int n = 0; n < 4; n++){
                    int nbr_id = nbrs[n];
                    if (nbr_id < 0) continue;
                    double dist = solver_map.heuristics[g_id][nbr_id];
                    if (dist < min_dist){
                        min_dist = dist;
                        min_nbr_id = nbr_id;
                    }
                }
                if (std::find(start_ids.begin(), start_ids.end(), min_nbr_id) == start_ids.end()){
                    std::clog << "Corrected overlapping start from "
                              << start_ids[robot]
                              << " to "
                              << min_nbr_id
                              << std::endl;
                    start_ids[robot] = min_nbr_id;
                    start_loc[robot].first = min_nbr_id;
                    num_correct++;
                }
                if (num_correct == overlapped_sets[i].size()-1){
                    break;
                }
            }
        }
    }
}

/* Get the direction number (Right:0, Down:1, Left:2, Up:3) from yaw value */
int MultiAgentRoutePlanner::getDirection(double yaw_){
    // Get Direction from yaw
    if (PI/4 <= yaw_ && yaw_ <= PI*3/4)
        return 0;   // Right: pi*1/4 ~ pi*3/4
    else if( (PI*3/4 <= yaw_ && yaw_ <= PI) || (-PI <= yaw_ && yaw_ <= -PI*3/4) )
        return 1;   // Down: pi*3/4 ~ pi || -pi*3/4 ~ -pi
    else if(-PI*3/4 <= yaw_ && yaw_ <= -PI/4)
        return 2;   // Left: -pi*3/4 ~ -pi*1/4
    else
        return 3;   // Up: -pi*1/4 ~ pi*1/4
}

/* Set robot stations for dummy path */
void MultiAgentRoutePlanner::setStations(){
    // We assume that the dock node is isolated for each robot
    // Therefore, we set the dock nodes as the goals of dummy path
    dummy_goal_loc.clear();
    for (int i = 0; i < topolomap->nodes_dock.size(); i++){
        int id = topolomap->nodes_dock[i];
        dummy_goal_loc.push_back(std::make_pair(id, 0));
        // Update heuristic information of the station node for A* path planning
        solver_map.heuristics[id] = path_planner->getHeuristics(solver_map, id);
    }
}

/* Distance b/w two yaw angles */
double MultiAgentRoutePlanner::getYawDist(double yaw1, double yaw2){
    double dist = yaw1 - yaw2;
    if (dist > 3.14159 )    dist -= 2.0 * 3.14;
    if (dist < -3.14159)    dist += 2.0 * 3.14;
    return std::abs(dist);
}

/* Get closet direction of yaw angle */
int MultiAgentRoutePlanner::getMinDir(double yaw, std::vector<double> yaws, std::vector<bool> check){
    int dir = -1;
    double min_dist_yaw = 9999;
    for (int i = 0; i < yaws.size(); i++){
        if (!check[i]) continue;
        double dist_yaw = getYawDist(yaw, yaws[i]);
        if (dist_yaw < min_dist_yaw){
           dir = i;
           min_dist_yaw = dist_yaw;
        }
    }
    return dir;
}

void MultiAgentRoutePlanner::setModes(int fms_mode_){
    fms_mode = fms_mode_;
}

}
