// Route-table bookkeeping and dependency management shared by traffic control and route planning.

#include <fleet_manager/route_table.h>

namespace FMS
{
/* Set parameters for route table */

Table RouteTable::copyRouteTable() {
    Table copied_route_table;

    //  route_table   
    copied_route_table.resize(route_table.size());

    //    
    for (int i = 0; i < route_table.size(); i++) {
        for (int j = 0; j < route_table[i].size(); j++) {
            NodeRT* original_node = route_table[i][j];

            // NodeRT  
            NodeRT* copied_node = new NodeRT(*original_node);

            // preconditions 
            for (NodeRT* precondition : original_node->preconditions) {
                copied_node->preconditions.push_back(precondition);
            }

            //    route_table 
            copied_route_table[i].push_back(copied_node);
        }
    }

    return copied_route_table;
}


void RouteTable::setParams(int num_of_robots_, bool add_init_table_, int init_step_size_){
    num_of_robots   = num_of_robots_;       // Number of robots
    add_init_table  = add_init_table_;      // True: appending initial route table for every planning step
    init_step_size  = init_step_size_;      // Step size for initial route table (desired set)
    route_table.resize(num_of_robots);      // Main route table
    current_steps.resize(num_of_robots);    // Current steps in route table
    committed_steps.resize(num_of_robots);  // Committed steps for  initial route table
}

/* Generate an initial route table by computing a commit cut in route table (ref: Hnig et al. 2018) */
void RouteTable::setInitRouteTable(std::vector<bool> need_replanning, std::vector<int> loc_ids){
    if (add_init_table){
        getCommittedSteps();                                        // 1) Get committed steps in route table
        computeInitRouteTable();                                    // 2) Compute an initial route table by cutting up to committed steps
        clearRouteTableOfSelectedRobots(need_replanning, loc_ids);  // 3) Clear all nodes for robots whose paths need to be replanned
    }
    else {
        // Do nothing
    }
}

/*  Clear current route table  */
void RouteTable::clearRouteTable(){
    releaseRouteTableNodes(true);
}

/* Compute the committed steps (ref: Hnig et al. 2018) */
void RouteTable::getCommittedSteps(){

    // 1) Get the current steps (current node index of each robot)
    current_steps.clear();
    current_steps.resize(num_of_robots, -1);
    for (int i = 0; i < num_of_robots; i++){
        for (int t = 0; t < route_table[i].size(); t++){
            NodeRT* node = route_table[i][t];
            if (node->state ==2 || node->state == 1 || node->state == 0){      // First visit to staged(0) or enqueued(1) or in-progress(2) node
                current_steps[i] = t;
                break;
            }
        }
    }

    // 2) Compute the desired steps
    std::vector<int> desired_steps(num_of_robots, -1);
    for (int i = 0; i < num_of_robots; i++){
        if (current_steps[i] < 0) continue;
        // A desired step is simply determined by (current step + initial step size)
        int step = std::min(current_steps[i] + init_step_size, (int)route_table[i].size() - 1);
        desired_steps[i] = step;
    }

    // 3) Get the committed nodes (ref: Algorithm 2-Compute Commit Cut in Hnig et al. 2018)
    std::queue<std::pair<int,int>> queue;
    std::vector<std::vector<bool>> visited(num_of_robots);
    for (int i = 0; i < num_of_robots; i++){
        if (desired_steps[i] < 0) continue;
        std::pair<int,int> node_i(i, desired_steps[i]);
        queue.push(node_i);
        visited[i].resize(route_table[i].size(), false);
        visited[i][desired_steps[i]] = true;
    }
    std::vector<std::pair<int,int>> reachable;
    while (queue.size() > 0){
        std::pair<int,int> node_i = queue.front();
        queue.pop();
        reachable.push_back(node_i);

        if (node_i.second != 0 && !visited[node_i.first][node_i.second-1]){
            std::pair<int,int> node_j(node_i.first, node_i.second-1);
            queue.push(node_j);
            visited[node_i.first][node_i.second-1] = true;
        }

        NodeRT* node = route_table[node_i.first][node_i.second];
        for (int k = 0; k < node->preconditions.size(); k++){
            int rid  = node->preconditions[k]->robot_id;
            int step = node->preconditions[k]->timestep;
            if (!visited[rid][step]){
                std::pair<int,int> node_j(rid, step);
                queue.push(node_j);
                visited[rid][step] = true;
            }
        }
    }

    // 4) Finally update the committed steps
    committed_steps.clear();
    committed_steps.resize(num_of_robots, -1);
    for (int i = 0; i < reachable.size(); i++){
        int rid  = reachable[i].first;
        int step = reachable[i].second;
        if (committed_steps[rid] < step){
            committed_steps[rid] = step;
        }
    }
}

/* Generate the initial route table by cutting up to the committed steps */
void RouteTable::computeInitRouteTable(){

    // The initial route table is generated
    // by inserting nodes from a current step to a committed step for each robot
    Table init_route_table;
    init_route_table.resize(num_of_robots);
    for (int i = 0; i < num_of_robots; i++){
        for (int t = 0; t < route_table[i].size(); t++){
            NodeRT* node = route_table[i][t];
            // Inserting nodes from a current step to a committed step
            if (t >= current_steps[i] && t <= committed_steps[i])
                init_route_table[i].push_back(node);
            else route_table[i][t]->remove = true;  // Delete the other nodes
        }
    }
    // Assign new timesteps in initial route table
    for (int i = 0; i < num_of_robots; i++){
        for (int t = 0; t < init_route_table[i].size(); t++){
            NodeRT* v = init_route_table[i][t];
            v->timestep = t;
            v->init_node = true;
        }
    }

    updatePreconditions(init_route_table);  // Remove preconditions pointing to deleted nodes
    releaseRouteTableNodes(false);          // Release all nodes labelled as (remove = true)
    route_table = init_route_table;         // Update the route table to initial route table

}

/* Clear all nodes for robots whose paths need to be replanned */
void RouteTable::clearRouteTableOfSelectedRobots(std::vector<bool> need_replanning, std::vector<int> loc_ids){

    // Set remove true  for replanning robots
    for (int i = 0; i < num_of_robots; i++){
        if (!need_replanning[i]) {
            loc_ids[i] = -1;
            continue;
        }
        for (int t = 0; t < route_table[i].size(); t++){
            NodeRT* v = route_table[i][t];
            v->remove = true;
        }
    }

    // If a robot's path passes through a start location of a replanning robot,
    // remove a sub-path from the start location.
    std::vector<NodeRT*> last_node(num_of_robots);
    for (int i = 0; i < num_of_robots; i++){
        if (need_replanning[i]) continue;
        if (route_table[i].size() == 0){
            last_node[i] = nullptr;
            continue;
        }
        bool found = false;
        for (int t = 0; t < route_table[i].size(); t++){
            NodeRT* v = route_table[i][t];
            if (!found){
                // Check a path passes through a start location.
                if (std::find(loc_ids.begin(), loc_ids.end(), v->id_end) != loc_ids.end()){
                    // If it passes a start location, remove this node
                    v->remove = true;
                    found = true;
                    continue;
                }
                last_node[i] = v; // Set the last node
            }
            else { v->remove = true; }
        }
        if (!found){
            // Set the last node
            last_node[i] = route_table[i].back();
        }
    }

    bool updated = true;
    while (updated){
        updated = false;
        for (int i = 0; i < num_of_robots; i++){
            if (need_replanning[i]) continue;
            bool found = false;
            for (int t = 0; t < route_table[i].size(); t++){
                NodeRT* v = route_table[i][t];
                if (v->remove) break;
                if (!found){
                    for (int k = 0; k < v->preconditions.size(); k++){
                        int rid = v->preconditions[k]->robot_id;
                        if (need_replanning[rid]) continue;
                        // If a node's precondition is removed, we remove the node
                        if (v->preconditions[k]->remove || v->preconditions[k]->id_end == last_node[rid]->id_end){
                            int t_prev = std::max(t-1, 0);
                            last_node[i] = route_table[i][t_prev];
                            v->remove = true;
                            found = true;
                            updated = true;
                            break;
                        }
                    }
                }
                else { v->remove = true; }
            }
        }
    }

    // Generate new route table by rejecting removing nodes
    Table new_route_table;
    new_route_table.resize(num_of_robots);
    for (int i = 0; i < num_of_robots; i++){
        for (int t = 0; t < route_table[i].size(); t++){
            NodeRT* v = route_table[i][t];
            if (!v->remove){
                new_route_table[i].push_back(v);
            }
            else { break; }
        }
    }

    updatePreconditions(new_route_table);   // Remove preconditions pointing to deleted nodes
    releaseRouteTableNodes(false);          // Release all nodes labelled as (remove = true)
    route_table = new_route_table;          // Update the route table to new route table
}

/* Remove preconditions pointing to deleted nodes
  (Remove the dependency relations (edges) of deleted nodes) */
void RouteTable::updatePreconditions(Table& new_route_table){
    for (int i = 0; i < num_of_robots; i++){
        for (int t = 0; t < new_route_table[i].size(); t++){
            NodeRT* v = new_route_table[i][t];
            for (auto it = v->preconditions.begin(); it != v->preconditions.end();) {
                if ((*it)->remove)                          // If this is a deleted node
                     it = v->preconditions.erase(it);       // Remove this precondition
                else it++;
            }
        }
    }
}

/* Get the start locations for MAPF
  (if we use the initial table, set the start locations to the last nodes of initial route table) */
std::vector<std::pair<bool, Eigen::Vector3d>> RouteTable::getStartNodes(){
    // The first compoent of the pair <bool, Eigen::Vector3d>
    //   1) true: it is updated to the last nodes of initial route table
    //   2) false: the start location will be updated to robot's current location
    std::vector<std::pair<bool, Eigen::Vector3d>> start_nodes;
    for (int i = 0; i < num_of_robots; i++) {
        Eigen::Vector3d start_pose;
        bool is_updated;
        if (add_init_table && route_table[i].size() > 0){
            start_pose = route_table[i].back()->end;        // Last node will be a start location
            is_updated = true;
        }
        else {
            start_pose = Eigen::Vector3d(-1, -1, -1);       // Else, we will not set the start node
            is_updated = false;
        }
        start_nodes.emplace_back(is_updated, start_pose);
    }
    return start_nodes;
}

/*  Set a new route table based on the computed multi-agent paths */
void RouteTable::setRouteTable(Routes routes, std::vector<int> loc_ids, std::vector<Eigen::Vector3d> curr_poses){

    // 1) Generate nodes of route table based on routes
    Table curr_route_table(num_of_robots);              // Current route table

    for (int i = 0; i < num_of_robots; i++){
        int goal_index = -1;
        const int route_size = static_cast<int>(routes[i].size());
        for (int t = 0; t + 1 < route_size; t++){

            NodeRT* node = new NodeRT();
            node->robot_id = i;                         // Robot id
            node->id_start = routes[i][t].loc_id;       // Start node id
            node->id_end   = routes[i][t+1].loc_id;     // End node id
            node->start    = routes[i][t].pose;         // Start pose
            node->end      = routes[i][t+1].pose;       // End pose
            node->timestep = t;                         // Time step
            node->state    = 0;                         // Node state: [0: staged], [1: enqueued], [2: in-progress], [3: finished], [4: dummy node], [5: occupied] [6: idle]
            node->init_node = false;                    // This is not an initial node
            node->is_waypoint = routes[i][t+1].is_waypoint;
            if(t + 1 == route_size - 1) node->is_waypoint = true;
            if (routes[i][t].is_dummy || routes[i][t+1].is_dummy){
                node->state = 4;                        // If this is a dummy node, set the state to 4
            }
            else { goal_index = t; }
            curr_route_table[i].push_back(node);        // Insert this node to route table

            // std::cout << "(" << node->state << ")";
        }
        if (goal_index >= 0){
            curr_route_table[i][goal_index]->is_goal = true;
        }

        // std::cout << routes[i][routes[i].size()-1].loc_id << std::endl;

    }

    // 2) Remove dummy nodes
    // Compute [Counter: minimum node index that overlaps with a dummy node]
    std::vector<int> counter(num_of_robots, -1);
    for (int i = 0; i < num_of_robots; i++){
        bool found = false;
        for (int j = 0; j < curr_route_table[i].size(); j++){
            NodeRT* curr_n = curr_route_table[i][j];
            if (curr_n->state == 4){                    // If current node is dummy node,
                found = true;                           // the counter of the robot is found
            }
            else {
                for (int k = 0; k < num_of_robots; k++){
                    if (k == i) continue;
                    for (int s = std::min(j, (int)curr_route_table[k].size()-1); s >= 0; s--){
                        NodeRT* prev_n = curr_route_table[k][s];
                        if (prev_n->state == 4){
                            if (prev_n->id_start == curr_n->id_end ||prev_n->id_end == curr_n->id_end){
                                // If a previous node overlaps with a dummy node,
                                // the counter of the robot is found
                                found = true;
                                break;
                            }
                        }
                    }
                    if (found) break;
                }
            }
            if (found){         // If we found the condition,
                counter[i] = j; // set the current index to counter
                break;
            }
        }
    }



    // Remove dummy nodes: release nodes starting from the counter in current route table
    for (int i = 0; i < num_of_robots; i++){
        if (counter[i] >= 0) {
            for (int t = counter[i]; t < curr_route_table[i].size(); t++){
                delete curr_route_table[i][t];
            }
            curr_route_table[i].resize(counter[i]);
        } else if (!routes[i].empty()) {
            continue;
        }

        if(curr_route_table[i].empty()) {
            // std::cout << "======================================No route for robot " << i <<"No route for robot======================================"<< std::endl;
            NodeRT* node = new NodeRT();
            node->robot_id = i;                         // Robot id
            node->id_start = loc_ids[i];       // Start node id
            node->id_end   = loc_ids[i];     // End node id
            node->start    = curr_poses[i];         // Start pose
            node->end      = curr_poses[i];       // End pose
            node->timestep = 0;                         // Time step
            node->state    = 4;                         // Node state: [0: staged], [1: enqueued], [2: in-progress], [3: finished], [4: dummy node], [5: occupied]
            node->init_node = false;                    // This is not an initial node
            node->is_waypoint = true;
            curr_route_table[i].push_back(node);        // Insert this node to route table
        }

        else{
            curr_route_table[i][0]->is_waypoint = true; // Set the first node as a waypoint node
            // curr_route_table[i].back()->is_waypoint = true; // Set the last node as a waypoint node
            if(curr_route_table[i][curr_route_table[i].size() - 1]->id_start == curr_route_table[i][curr_route_table[i].size() - 1]->id_end){
                for(int r = curr_route_table[i].size() - 1; r >= 0; r--){
                    curr_route_table[i][r]->is_waypoint = true; // Set the last node as a waypoint node
                    if(curr_route_table[i][r]->id_start != curr_route_table[i][r]->id_end){
                        break;
                    }
                }
            }
        }
    }



    // Set the route table only with waypoint nodes
    std::vector<std::vector<NodeRT>> wp_route_table(num_of_robots);
    for (int i = 0; i < num_of_robots; i++){
        uint32_t prev_id_end;
        Eigen::Vector3d prev_end;
        bool is_assigned = false;

        for (int t = 0; t < curr_route_table[i].size(); t++){
            NodeRT node = *curr_route_table[i][t];
            // TODO :   vertex waypoint   
            if(node.is_waypoint || (t == 0 || t == curr_route_table[i].size() - 1)){
                if(is_assigned){
                    node.id_start = prev_id_end;
                    node.start = prev_end;
                    node.is_waypoint = true; // If the node is not a waypoint node, set it as a waypoint node
                }
                else{
                    node.id_start = curr_route_table[i][0]->id_start;
                    node.start = curr_route_table[i][0]->start;
                    node.is_waypoint = true; // If the node is not a waypoint node, set it as a waypoint node
                }

                wp_route_table[i].push_back(node);

                prev_id_end = node.id_end;
                prev_end = node.end;

                is_assigned = true;

            }
            else{
                wp_route_table[i].push_back(node);
            }
        }

    }

    // 3) Set the preconditions of each node
    for (int i = 0; i < num_of_robots; i++){
        for (int j = 0; j < wp_route_table[i].size(); j++){
            if(wp_route_table[i][j].state == 4) continue;
            NodeRT curr_v = wp_route_table[i][j];

            for (int k = 0; k < num_of_robots; k++){
                if (k == i) continue;
                for (int s = std::min(j, (int)wp_route_table[k].size()-1) ; s >= 0 ; s--){
                    if(wp_route_table[k][s].id_start == wp_route_table[k][s].id_end && wp_route_table[k][s].state != 4) { continue; }

                    NodeRT prev_v = wp_route_table[k][s];
                    // If robots i and k pass the same segment (start or end node)

                    // if (prev_v.id_start == curr_v.id_end || prev_v.id_end == curr_v.id_end){
                    //     // And if both nodes are waypoint nodes, update the precondition
                    //     if(curr_v.is_waypoint || prev_v.is_waypoint){
                    //         curr_route_table[i][j]->preconditions.push_back(curr_route_table[k][s]);
                    //         break;
                    //     }
                    // }



                    if (prev_v.id_start == curr_v.id_end || prev_v.id_end == curr_v.id_end){
                        // And if both nodes are waypoint nodes, update the precondition
                        if(curr_v.is_waypoint || prev_v.is_waypoint){
                            curr_route_table[i][j]->preconditions.push_back(curr_route_table[k][s]);
                            break;
                        }
                    }
                    // else if((s == (int)wp_route_table[k].size()-1 || j == (int)wp_route_table[i].size()-1) && prev_v.id_end == curr_v.id_end){
                    //     if((curr_v.is_waypoint || prev_v.is_waypoint) && prev_v.state != 4){
                    //         curr_route_table[i][j]->preconditions.push_back(curr_route_table[k][s]);
                    //         break;
                    //     }
                    // }
                }
            }
        }
    }

    if(fms_mode == 1){
        edges.clear();
        std::vector<std::vector<LineInfo>> overlap_lines = getOverlapLines(wp_route_table);   // get overlapping lines

        for(auto lines: overlap_lines) {
            // Sort the lines in the lines by their start time
            std::sort(lines.begin(), lines.end(), [](const LineInfo &a, const LineInfo &b) {
                return a.line_start < b.line_start;
            });

            int edge_size = lines.size() - 1;
            for(int i = 0; i < lines.size(); i++) {
                LineInfo start = lines[i];

                for(int j = 0; j < lines.size(); j++) {
                    if(lines[j].robot_id == start.robot_id && lines[j].line_start == start.line_start) {
                        lines.erase(lines.begin() + j);
                        break;
                    }
                }

                if(set_preconditions(curr_route_table, lines, start, edge_size)) {
                    break;
                }

                lines.push_back(start);
            }
        }

        for(auto edge: edges) {
            LineInfo start = edge.first;
            LineInfo end = edge.second;
            curr_route_table[end.robot_id][end.line_start]->preconditions.push_back(curr_route_table[start.robot_id][start.line_start]);
        }
    }



    // 4) Set the current route table
    if (!add_init_table){
        // If we do not use an initial route table, just replace the route table
        releaseRouteTableNodes(true);
        // route_table = curr_route_table;
        route_table.resize(curr_route_table.size());

        for (size_t i = 0; i < curr_route_table.size(); ++i) {
            for (size_t j = 0; j < curr_route_table[i].size(); ++j) {
                NodeRT* original_node = curr_route_table[i][j];

                // Create a new NodeRT object by copying the original
                NodeRT* copied_node = new NodeRT(*original_node);

                // Deep copy the preconditions
                copied_node->preconditions.clear();
                for (NodeRT* precondition : original_node->preconditions) {
                    copied_node->preconditions.push_back(new NodeRT(*precondition));
                }

                // Add the copied node to the route_table
                route_table[i].push_back(copied_node);
            }
        }

    }
    else {
        // Todo: Implement the case when we use an initial route table
        // If we use an initial route table, update the preconditions for initial nodes
        for (int i = 0; i < num_of_robots; i++){
            for (int j = 0 ; j < curr_route_table[i].size() ; j++){
                NodeRT* curr_v = curr_route_table[i][j];
                for (int k = 0; k < num_of_robots; k++){
                    if (k == i) continue;
                    for (int s = (int)route_table[k].size()-1; s >= 0; s--){
                        NodeRT* prev_v = route_table[k][s];
                        // If robots i and k pass the same segment (start or end node), update the precondition

                        // if (prev_v->id_start == curr_v->id_end || prev_v->id_end == curr_v->id_end){
                        //     // Update preconditions
                        //     curr_route_table[i][j]->preconditions.push_back(route_table[k][s]);
                        //     break;
                        // }


                        if (prev_v->id_start == curr_v->id_end){
                            // Update preconditions
                            curr_route_table[i][j]->preconditions.push_back(route_table[k][s]);
                            break;
                        }
                        else if((s == (int)route_table[k].size()-1 || (int)route_table[i].size()-1) && prev_v->id_end == curr_v->id_end){
                            curr_route_table[i][j]->preconditions.push_back(route_table[k][s]);
                            break;
                        }
                    }
                }
            }
        }

        // Merge initial route table and current route table
        for (int i = 0; i < num_of_robots; i++){
            int index = route_table[i].size();
            for (int t = 0; t < curr_route_table[i].size(); t++){
                curr_route_table[i][t]->timestep = index + t;
                route_table[i].push_back(curr_route_table[i][t]);
            }
        }
    }

}

/* Append the goal station nodes in route table if it removes the station nodes */
void RouteTable::setGoalStationNodes(std::vector<Eigen::Vector3d> goal_poses){
    // Insert the goal pose into the last goal segement
    for (int i = 0; i < num_of_robots; i++){
        if (route_table[i].size() == 0) continue;

        NodeRT* last_node = route_table[i].back();
        if (last_node->is_goal){
            NodeRT* node = new NodeRT();
            node->robot_id = i;                         // Robot id
            node->id_start = last_node->id_start;       // Start node id
            node->id_end   = last_node->id_end;         // End node id
            node->start    = last_node->end;            // Start pose
            node->end      = goal_poses[i];             // End pose
            node->timestep = last_node->timestep + 1;   // Time step
            node->state    = 0;                         // Node state
            node->init_node = false;                    // This is not an initial node
            node->is_waypoint = last_node->is_waypoint;
            node->preconditions = last_node->preconditions;
            route_table[i].push_back(node);             // Insert this node to route table
        }
    }
}

/* Update the node states in route table */
void RouteTable::updateRouteStates(std::vector<Eigen::Vector2d> poses){
    // TODO: update compute current steps
    // Compute the current steps (current node indices) based current locations
    std::vector<int> curr_steps(num_of_robots, 0);
    for (int i = 0; i < num_of_robots; i++){
        float min_dist = 99999;
        float wp_min_dist = 99999;
        int wp_min_step = -1;
        for (int t = curr_steps[i]; t < route_table[i].size(); t++){
            NodeRT* node = route_table[i][t];
            // if(!node->is_waypoint && (t < route_table[i].size()-1 || t!=0)) continue;
            if (node->state == 4) break;

            double dist1 = (node->start.head(2) - poses[i]).norm();
            double dist2 = (node->end.head(2) - poses[i]).norm();
            double d = dist1 + dist2;
            // if(node->is_waypoint && d < wp_min_dist){
            //     wp_min_step = t;
            //     wp_min_dist = d;
            // }
            if (d < min_dist){      // Get the closet node
                curr_steps[i] = t;
                min_dist = d;
            }
            else if(d == min_dist && t-1 == curr_steps[i]){
                curr_steps[i] = t;
            }
        }
    }

    // Update the node states in route table
    bool updated = true;
    while (updated){
        updated = false;
        for (int i = 0; i < num_of_robots; i++){
            bool in_progress = false;
            for (int t = 0; t < route_table[i].size(); t++){
                NodeRT* node = route_table[i][t];
                if (node->state == 0){      // 0: staged
                    bool can_pass = true;
                    for (int k = 0; k < node->preconditions.size(); k++){
                        // If there is a precondition node whose state is not (3: finished),
                        // the robot cannot pass this node
                        if (node->preconditions[k]->state != 3){
                            // if(node->preconditions[k]->state == 3 || node->preconditions[k]->state == 1){
                            //     if(!node->preconditions[k]->is_goal || !node->is_goal)node->state = 5;
                            // }
                            can_pass = false;
                            break;
                        }
                    }
                    if (can_pass) {
                        node->state = 1;
                        updated = true;

                    }
                    else          { break; }
                }
                if (node->state == 1){      // 1: enqueued
                    if(!node->is_waypoint){
                        if(t <= curr_steps[i]){
                            node->state = 2;
                            updated = true;
                            in_progress = true;
                        }
                    }
                    if(t <= curr_steps[i] && !in_progress){ // If current step equals with this node,
                        node->state = 2;    // this node is in-progress state
                        updated = true;
                        in_progress = true;
                    }
                }
                if (node->state == 2 || node->state == 1){  // 2: In-progress 1: enqueued
                    if (t < curr_steps[i]){                 // If current step is grater than this node,
                        node->state = 3;                    // this node is already passed
                        updated = true;
                    }
                    // else if((node->end.head(2) - poses[i]).norm() < 0.1 && t <= curr_steps[i]){
                    //     node->state = 3;
                    //     updated = true;
                    // }
                }
                if (node->state == 3){      // 3: finished
                    // Do nothing
                }
            }
        }
    }
}

/* Get a global route of a robot from route table*/
std::vector<Eigen::Vector3d> RouteTable::getGlobalRoute(int robot_i){
    std::vector<Eigen::Vector3d> global_route;
    for (int t = 0; t < route_table[robot_i].size(); t++){
        global_route.push_back(route_table[robot_i][t]->end);
    }
    return global_route;
}

/* Setting is_waypoint based on end */
std::vector<Eigen::Vector3d> RouteTable::getLocalRoute(int robot_i, std::vector<Eigen::Vector2d> curr_poses){
    Eigen::Vector2d curr_pose = curr_poses[robot_i];
    std::vector<Eigen::Vector3d> local_route;
    Eigen::Vector3d prev_end;
    bool is_assigned = false;


    int last_inq = -1;
    // std::cout << "robot_i " << robot_i << "  " << std::endl;


    if(fms_mode == 2){
        NodeRT* node = route_table[robot_i][route_table[robot_i].size()-1];
        double yaw;
        if (curr_pose == node->end.head(2)){ return local_route; }
        else {
            Eigen::Vector2d vec = node->end.head(2) - curr_pose;
            yaw  = std::atan2(vec[1], vec[0]);
        }
        Eigen::Vector3d pt = node->end;
        pt[2] = yaw;
        local_route.push_back(pt);
    }

    else if(fms_mode == 0){
        int curr_loc = 0;
        for (int t = 0; t < route_table[robot_i].size(); t++){
            NodeRT* node = route_table[robot_i][t];
            if      (node->state == 0){ break; }
            else if (node->state == 1|| node->state == 2){
                curr_loc = node->id_start;
                double yaw;
                // if (node->id_start == node->id_end){
                if (node->start.head(2) == node->end.head(2)){ continue; }
                else {
                    Eigen::Vector2d vec = node->end.head(2) - node->start.head(2);
                    yaw  = std::atan2(vec[1], vec[0]);
                }
                Eigen::Vector3d pt = node->end;
                pt[2] = yaw;
                local_route.push_back(pt);
            }
            else if(node->state == 4 && node->timestep == 0){
                curr_loc = node->id_start;
                double yaw;
                Eigen::Vector2d vec = node->start.head(2) - curr_poses[robot_i];
                yaw  = std::atan2(vec[1], vec[0]);
                Eigen::Vector3d pt = node->start;
                pt[2] = yaw;
                local_route.push_back(pt);
                break;
            }
            else if(node->state == 3){
                // If the node is finished, set the current location to the end of the node
                curr_loc = node->id_end;
            }
        }
        if(curr_loc == 0 && !local_route.empty()){
            curr_loc = route_table[robot_i][0]->id_start;
        }
        if(local_route.empty() && route_table[robot_i].size() > 0){
            int curr_step = 0;
            for(int t = 0; t < route_table[robot_i].size(); t++){
                if(route_table[robot_i][t]->state != 3){
                    curr_step = t;
                    break;
                }
            }
            NodeRT* node = route_table[robot_i][curr_step];
            double yaw;
            Eigen::Vector2d vec = node->start.head(2) - curr_poses[robot_i];
            yaw  = std::atan2(vec[1], vec[0]);
            Eigen::Vector3d pt = node->start;
            pt[2] = yaw;
            local_route.push_back(pt);
        }

    }

    else{
        for(int t = 0; t < route_table[robot_i].size(); t++){
            NodeRT* node = route_table[robot_i][t];

            // ad hoc measure
            // if(t == 0) prev_end = node->end;
            if(node->is_waypoint || t == route_table[robot_i].size() - 1){
                if      (node->state == 0){
                    last_inq = t-1;
                    break;
                }


                else if (node->state == 1 || node->state == 2){
                    double yaw;

                    if(is_assigned){
                        if (prev_end.head(2) == node->end.head(2)){ continue; }
                        else {
                            Eigen::Vector2d vec = node->end.head(2) - prev_end.head(2);
                            yaw  = std::atan2(vec[1], vec[0]);
                        }
                    }
                    else{
                        if (curr_pose == node->end.head(2)){ continue; }
                        // if ((node->end.head(2) - curr_pose).norm() < 0.05) continue;
                        else {
                            Eigen::Vector2d vec = node->end.head(2) - curr_pose;
                            yaw  = std::atan2(vec[1], vec[0]);
                        }

                        is_assigned = true;
                    }

                    Eigen::Vector3d pt = node->end;
                    // std::cout << node->id_end << "[" << node->state << "]" <<"  ";
                    pt[2] = yaw;
                    local_route.push_back(pt);
                    prev_end = node->end;
                }
                else{ continue; }
            }
        }
        if(local_route.empty()  && last_inq >= 0){
            NodeRT* node = route_table[robot_i][last_inq];
            if (node->state == 1 || node->state == 2){
                double yaw;
                if (curr_pose == node->end.head(2)){ return local_route; }
                else {
                    Eigen::Vector2d vec = node->end.head(2) - curr_pose;
                    yaw  = std::atan2(vec[1], vec[0]);
                }
                Eigen::Vector3d pt = node->end;
                pt[2] = yaw;
                local_route.push_back(pt);
            }
        }
    }
    // std::cout << std::endl;
    return local_route;
}

/* Release (all or selected) nodes in route table */
void RouteTable::releaseRouteTableNodes(bool delete_all){

    for (int i = 0; i < route_table.size(); i++){
        for (auto it = route_table[i].begin(); it != route_table[i].end(); it++){
            if (delete_all || (*it)->remove){
                delete *it;
            }
        }
        route_table[i].clear();
    }
    route_table.clear();
}

void RouteTable::switchRouteTableNodes(std::vector<Eigen::Vector2d> poses){

    // Calculate the distance to the next node and switch the dependency
    for (int i = 0; i < num_of_robots; i++){
        for (int t = 0; t < route_table[i].size(); t++){
            NodeRT* node = route_table[i][t];

            if (node->state == 2){
                double d1 = (node->end.head(2) - poses[i]).norm();
                std::vector<NodeRT*> preconditions = node->preconditions;

                for(int j = 0; j < preconditions.size(); j++){
                    double d2 = (preconditions[j]->end.head(2) - poses[i]).norm();
                    if (d1 < d2){
                        int robot_id = preconditions[j]->robot_id;
                        int timestep = preconditions[j]->timestep;
                        route_table[robot_id][timestep]->preconditions.push_back(node);
                        node->preconditions[j]->remove = true;
                    }
                }
            }
            else if (node->state == 4) break;
        }
    }

    updatePreconditions(route_table);
}

fms_msgs::msg::RouteTable RouteTable::getRouteTable() {

    fms_msgs::msg::RouteTable pub_route_table;

    for(int i = 0; i < route_table.size(); i++) {

        fms_msgs::msg::AgentRouteTable agent_route_table;

        for(int j = 0; j < route_table[i].size(); j++) {
            fms_msgs::msg::RouteTableNode route_table_node;

            route_table_node.nodert.robot_id    = route_table[i][j]->robot_id;
            route_table_node.nodert.id_end      = route_table[i][j]->id_end;
            route_table_node.nodert.id_start    = route_table[i][j]->id_start;
            route_table_node.nodert.timestep    = route_table[i][j]->timestep;
            route_table_node.nodert.state       = route_table[i][j]->state;

            route_table_node.nodert.remove      = route_table[i][j]->remove;
            route_table_node.nodert.init_node   = route_table[i][j]->init_node;
            route_table_node.nodert.is_goal     = route_table[i][j]->is_goal;
            route_table_node.nodert.is_waypoint = route_table[i][j]->is_waypoint;

            for(int k = 0; k < route_table[i][j]->preconditions.size(); k++) {
                fms_msgs::msg::NodeRT nodert;

                nodert.robot_id    = route_table[i][j]->preconditions[k]->robot_id;
                nodert.id_end      = route_table[i][j]->preconditions[k]->id_end;
                nodert.id_start    = route_table[i][j]->preconditions[k]->id_start;
                nodert.timestep    = route_table[i][j]->preconditions[k]->timestep;
                nodert.state       = route_table[i][j]->preconditions[k]->state;

                nodert.remove      = route_table[i][j]->preconditions[k]->remove;
                nodert.init_node   = route_table[i][j]->preconditions[k]->init_node;
                nodert.is_goal     = route_table[i][j]->preconditions[k]->is_goal;
                nodert.is_waypoint = route_table[i][j]->preconditions[k]->is_waypoint;

                route_table_node.preconditions.push_back(nodert);
            }

            agent_route_table.agent_route_table.push_back(route_table_node);
        }

        pub_route_table.route_table.push_back(agent_route_table);
    }

    return pub_route_table;
}

std::vector<std::vector<LineInfo>> RouteTable::getOverlapLines(const std::vector<std::vector<NodeRT>>& wp_route_table) {
    std::vector<std::vector<LineInfo>> overlap_lines;

    // Extract line information from each robot's route and group them by id
    std::map<int, std::vector<LineInfo>> group_by_id;
    for (int i = 0; i < num_of_robots; i++) {
        bool has_started = false;
        int start_timestep = 0;

        for (const auto& node : wp_route_table[i]) {
            if ((node.is_waypoint && (node.id_start != node.id_end)) || node.timestep == 0) {
                if (!has_started) {
                    start_timestep = node.timestep;
                    has_started = true;
                } else {
                    int end_timestep = node.timestep;
                    int line_id = node.id_start;
                    group_by_id[line_id].push_back({start_timestep, end_timestep, i, line_id});
                    start_timestep = end_timestep;
                }
            }
        }
    }

    // For each group of lines, apply the sweep line algorithm to capture accurate overlapping intervals
    for (const auto& [id, group] : group_by_id) {
        if (group.size() > 1) {
            std::vector<std::tuple<int, bool, LineInfo>> events; // time, is_start, line

            for (const auto& line : group) {
                events.push_back({line.line_start, true, line});
                events.push_back({line.line_end, false, line});
            }

            // Sort events in time order (if times are equal, process end events first)
            std::sort(events.begin(), events.end(),
                [](auto const& a, auto const& b) {
                    return std::tie(std::get<0>(a), std::get<1>(a)) < std::tie(std::get<0>(b), std::get<1>(b));
                }
            );

            bool is_added = false;
            std::vector<LineInfo> active; // Track currently active lines

            for (const auto& [time, is_start, line] : events) {
                if (is_start) {
                    active.push_back(line);
                    is_added = true;
                }
                else {
                    if(active.size() > 1 && is_added) {
                        overlap_lines.push_back(active);
                    }

                    for(int i = 0; i < active.size(); i++) {
                        if(active[i].robot_id == line.robot_id && active[i].line_start == line.line_start) {
                            active.erase(active.begin() + i);
                            is_added = false;
                            break;
                        }
                    }
                }
            }
        }
    }

    return overlap_lines;
}

std::vector<std::pair<int, int>> RouteTable::getSuccessors(const Table& curr_route_table, int robot_id, int timestep) {
    std::vector<std::pair<int, int>> successors;

    // Add sequential connection: same layer arrow from current node to the next node
    if (timestep + 1 < curr_route_table[robot_id].size()) {
        successors.emplace_back(robot_id, timestep + 1);
    }

    // Iterate over all nodes to check for precondition-based connections.
    for (int i = 0; i < curr_route_table.size(); i++) {
        for (int j = 0; j < curr_route_table[i].size(); j++) {
            // Check the preconditions of node curr_route_table[i][j]
            for (const auto &precondition : curr_route_table[i][j]->preconditions) {
                if (precondition->robot_id == robot_id && precondition->timestep == timestep) {
                    successors.emplace_back(i, j);
                    break;  // Prevent duplicate addition
                }
            }
        }
    }
    return successors;
}

bool RouteTable::is_cyclic(const Table& curr_route_table, int start_robot_id, int start_timestep, int target_robot_id, int target_timestep){
    // 2 dim vector to keep track of visited nodes
    std::vector<std::vector<bool>> visited(curr_route_table.size());
    for (int i = 0; i < curr_route_table.size(); i++) {
        visited[i].resize(curr_route_table[i].size(), false);
    }

    // stack to perform DFS
    std::stack<std::pair<int,int>> dfs;
    dfs.push({start_robot_id, start_timestep});
    visited[start_robot_id][start_timestep] = true;

    while (!dfs.empty()) {
        auto [curr_robot, curr_timestep] = dfs.top();
        dfs.pop();

        // return true if the target node is reached
        if (curr_robot == target_robot_id && curr_timestep == target_timestep) {
            return true;
        }

        // check nodes connected from the current node
        auto successors = getSuccessors(curr_route_table, curr_robot, curr_timestep);
        for (auto &successor : successors) {
            int next_robot = successor.first;
            int next_timestep = successor.second;
            if (!visited[next_robot][next_timestep]) {
                visited[next_robot][next_timestep] = true;
                dfs.push({next_robot, next_timestep});
            }
        }
    }

    // return false if the target node is not reachable
    return false;
}

int RouteTable::get_current_action(int agent){


    if(route_table[agent].size() == 0) return -1;
    int current_action = -1;
    for(int i = 0; i < route_table[agent].size(); i++) {
        if(route_table[agent][i]->state == 0 || route_table[agent][i]->state == 1) {
            return route_table[agent][i]->id_start;
        }
        else if(route_table[agent][i]->state == 4 && route_table[agent][i]->timestep == 0) {
            return route_table[agent][0]->id_start;
        }
        else if(route_table[agent][i]->state == 2) {
            return route_table[agent][0]->id_end;
        }
    }
    // if(current_action == 0) return -1;
    // std::cout << "Current Steps: " << current_action << std::endl;
    // std::cout << "Current Action ID: " << route_table[agent][current_action]->id_start << std::endl;
    return -1;
}

bool RouteTable::doIntersect(Eigen::Vector2d p1, Eigen::Vector2d q1, Eigen::Vector2d p2, Eigen::Vector2d q2) {
    auto orientation = [](Eigen::Vector2d p, Eigen::Vector2d q, Eigen::Vector2d r) {
        double val = (q[1] - p[1]) * (r[0] - q[0]) - (q[0] - p[0]) * (r[1] - q[1]);
        if (val == 0) return 0;  // collinear
        return (val > 0) ? 1 : 2; // clock or counterclock wise
    };

    int o1 = orientation(p1, q1, p2);
    int o2 = orientation(p1, q1, q2);
    int o3 = orientation(p2, q2, p1);
    int o4 = orientation(p2, q2, q1);

    // General case
    if (o1 != o2 && o3 != o4)
        return true;

    return false; // Doesn't fall in any of the above cases
}

bool RouteTable::set_preconditions(const Table& curr_route_table, std::vector<LineInfo> lines, LineInfo start, int edge_size) {
    if(edges.size() == edge_size) { return true; }

    std::vector<LineInfo> queue = lines;
    std::vector<LineInfo> queue_param = lines;

    while(!queue.empty()) {
        int min = 99999;
        int min_index = -1;
        LineInfo line;
        for(int i = 0; i < queue.size(); i++) {
            if(queue[i].line_start < min) {
                min = queue[i].line_start;
                line = queue[i];
                min_index = i;
            }
        }
        // Remove the line from the queue
        queue.erase(queue.begin() + min_index);

        if(!is_cyclic(curr_route_table, start.robot_id, start.line_end, line.robot_id, line.line_start)) {
            edges.push_back({start, line});
            queue_param.erase(queue_param.begin() + min_index);

            if(!set_preconditions(curr_route_table, queue_param, line, edge_size)) {
                queue_param.push_back(line);
                edges.erase(edges.end()-1);
            }
            else {
                return true;
            }
        }
    }

    return false;
}

void RouteTable::runESES(){
    Graph ESES_graph;
    ESES_graph.resize(num_of_robots);
    std::vector<int> curr_steps(num_of_robots, 0);
    // eses->ESES_graph.resize(num_of_robots);
    int empty_count = 0;

    for(int i = 0; i <  num_of_robots; i++){

        if(route_table[i].empty()){
            empty_count++;
            continue;
        }

        for(int t = 0; t < route_table[i].size(); t++){
            VertexRT* vertex = new VertexRT();
            vertex->robot_id = route_table[i][t]->robot_id;
            vertex->timestep = route_table[i][t]->timestep;

            vertex->is_satisfied = false;

            if((route_table[i][t]->state == 3 || route_table[i][t]->state == 2) || (route_table[i][t]->state == 4||route_table[i][t]->state == 6)){
                curr_steps[i] = t;
                vertex->is_satisfied = true; // If the node is already satisfied, set is_satisfied to true
            }

            vertex->is_waypoint = route_table[i][t]->is_waypoint;
            vertex->id_start = route_table[i][t]->id_start;
            vertex->id_end = route_table[i][t]->id_end;
            vertex->start = route_table[i][t]->start;
            vertex->end = route_table[i][t]->end;
            for (int p = 0; p < route_table[i][t]->preconditions.size(); p++){
                VertexRT* precondition_v = new VertexRT();
                precondition_v->robot_id = route_table[i][t]->preconditions[p]->robot_id;
                precondition_v->timestep = route_table[i][t]->preconditions[p]->timestep;
                precondition_v->is_satisfied = false;
                if((route_table[i][t]->preconditions[p]->state == 3 || route_table[i][t]->preconditions[p]->state == 2) || (route_table[i][t]->preconditions[p]->state == 4 || route_table[i][t]->preconditions[p]->state == 6)){
                    precondition_v->is_satisfied = true; // If the node is already satisfied, set is_satisfied to true
                }
                precondition_v->is_waypoint = route_table[i][t]->preconditions[p]->is_waypoint;
                precondition_v->id_start = route_table[i][t]->preconditions[p]->id_start;
                precondition_v->id_end = route_table[i][t]->preconditions[p]->id_end;
                precondition_v->start = route_table[i][t]->preconditions[p]->start;
                precondition_v->end = route_table[i][t]->preconditions[p]->end;

                if(optimizer == 1){
                    if(route_table[i][t]->state == 3 || route_table[i][t]->state == 2){
                    vertex->preconditions.push_back({precondition_v, true});
                    }
                    else if(route_table[i][t]->state == 4 || route_table[i][t]->preconditions[p]->state == 4){
                        vertex->preconditions.push_back({precondition_v, true});
                    }
                    else{
                        vertex->preconditions.push_back({precondition_v, false});
                    }
                }

                else if(optimizer == 0){
                    vertex->preconditions.push_back({precondition_v, true});
                }

                else {
                    std::cerr << "Unsupported route-table optimizer value: " << optimizer << std::endl;
                }
            }
            ESES_graph[i].push_back(vertex);
        }

    }

    if(empty_count == num_of_robots){ return; }
    auto eses = new ESES(ESES_graph, curr_steps, num_of_robots);

    if(optimizer == 1){
        auto sol = eses->runESES();
        double sol_cost = sol.second;

        rclcpp::Time current_time = rclcpp::Clock().now();
        double elapsed_time = (current_time-start_time).seconds();

        std::clog << "ESES cost: " << sol_cost << std::endl;

        std::ofstream csv_file(result_test_dir+"eses_res.csv", std::ios::app);

        if (csv_file.is_open()) {
            csv_file << elapsed_time << ", ESES cost, " << sol_cost << "\n";
            csv_file.close();
        } else {
            std::cerr << "Failed to open eses_res.csv" << std::endl;
        }

        for(int i = 0; i < num_of_robots; i++){

            if(route_table[i].empty()) continue;

            for (int t = 0; t < route_table[i].size(); t++){
                NodeRT* node = route_table[i][t];
                if(node->state == 3) continue;
                if(node->state <= 2) node->state = 0;
                node->preconditions.clear();
                for(int p = 0; p < sol.first[i][t]->preconditions.size(); p++){
                    // if(!sol.first[i][t]->preconditions[p].second) continue;
                    int d_agent = sol.first[i][t]->preconditions[p].first->robot_id;
                    int d_timestep = sol.first[i][t]->preconditions[p].first->timestep;
                    NodeRT* precondition_node = route_table[d_agent][d_timestep];
                    node->preconditions.push_back(precondition_node);
                }
            }
        }

    }

    else if(optimizer == 0){
        // double sol_cost = eses->get_costs();

        // rclcpp::Time current_time = rclcpp::Clock().now();
        // double elapsed_time = (current_time-start_time).seconds();

        // std::cout << "ESES cost: " << sol_cost << std::endl;
        // std::cout << "File: " << __FILE__ << ", ESES cost: " << sol_cost << std::endl;

        // std::ofstream csv_file(result_test_dir+"ADG_res.csv", std::ios::app);

        // if (csv_file.is_open()) {
        //     csv_file << elapsed_time << ", ESES cost, " << sol_cost << "\n";
        //     csv_file.close();
        // } else {
        //     std::cerr << "Failed to open eses_res.csv" << std::endl;
        // }

    }

    delete eses;

}

void RouteTable::set_modes(int fms_mode_, int optimizer_){
    fms_mode = fms_mode_;
    optimizer = optimizer_;
}

void RouteTable::set_result(std::string result_test_dir_){
    result_test_dir   = result_test_dir_;

}

}
