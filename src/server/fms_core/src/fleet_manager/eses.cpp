// ESES-based optimizer components used inside the multi-agent route planner.

#include <fleet_manager/eses.h>

namespace FMS
{

void ESES::init(const Graph& route_table, std::vector<int>& x){
  cost = 0;

  for(int i = 0; i < route_table.size(); i++){
    if(route_table[i].empty()) continue;
    int time_agent = x[i]+1;
    for(int t = 0; t < time_agent; t++){
      route_table[i][t]->is_satisfied = true;
      // x[i]++;
    }
  }
}

std::pair<double, bool> ESES::step(const Graph& route_table, std::vector<int>& timestep) {
  double cost_extend = 0.0;
  int complete = 0;
  // std::vector<bool> step_complete(route_table.size(), false);
  bool step_complete = true;

  std::vector<double> costs(route_table.size(), 0.0);
  std::vector<int> prev_step = timestep;

  for (int i = 0; i < route_table.size(); i++) {
    double agent_cost = 0.0;

    bool delay = false;

    if(costs[i] > 0.0) continue;

    if(route_table[i].empty()){
      complete++;
      continue;
    }

    if(route_table[i][route_table[i].size() - 1]->is_satisfied){
      complete++;
      continue;
    }

    if(timestep[i] >= route_table[i].size() - 1){
      for(int c = 0; c < route_table[i].size(); c++){
        if(route_table[i][c]->is_satisfied) continue;
        route_table[i][c]->is_satisfied = true;
      }
      timestep[i] = route_table[i].size();
      complete++;
      continue;
    }
    while (timestep[i] + 1 < route_table[i].size()-1 && !route_table[i][timestep[i] + 1]->is_waypoint) {
        route_table[i][timestep[i] + 1]->is_satisfied = true;
        timestep[i]++;
    }

    timestep[i] = std::min(timestep[i], (int)route_table[i].size()-2);


    if (timestep[i] + 1 < route_table[i].size() && route_table[i][timestep[i] + 1]->preconditions.empty()) {
      route_table[i][timestep[i] + 1]->is_satisfied = true;
      agent_cost = (route_table[i][prev_step[i]]->end.head(2) - route_table[i][timestep[i] + 1]->end.head(2)).norm();
      timestep[i]++;
      cost_extend += agent_cost;
      costs[i] = agent_cost;
    }

    else {
      // cost ...
      for (int j = 0; j < route_table[i][timestep[i] + 1]->preconditions.size(); j++) {
        auto precondition = route_table[i][timestep[i] + 1]->preconditions[j];
        if (!precondition.first) continue;

        int d_robot_id = precondition.first->robot_id;
        // int d_timestep = precondition.first->timestep;
        int d_timestep = std::min(prev_step[i], precondition.first->timestep);

        if(timestep[d_robot_id] >= route_table[d_robot_id].size()-1) continue;
        if(route_table[d_robot_id][d_timestep]->is_satisfied) continue;

        // if(d_timestep = timestep[d_robot_id] && costs[d_robot_id] > 0.0){

        //   if(timestep[d_robot_id] >= route_table[d_robot_id].size()-1 || route_table[d_robot_id].empty()) pass = true;
        //   else{
        //     while (timestep[d_robot_id] + 1 < route_table[d_robot_id].size()-1 && !route_table[d_robot_id][timestep[d_robot_id] + 1]->is_waypoint) {
        //         route_table[d_robot_id][timestep[d_robot_id] + 1]->is_satisfied = true;
        //         ++timestep[d_robot_id];
        //     }

        //     timestep[d_robot_id] = std::min(timestep[d_robot_id], (int)route_table[d_robot_id].size()-2);

        //     if (timestep[d_robot_id]+1 < route_table[d_robot_id].size() && route_table[d_robot_id][timestep[d_robot_id] + 1]->preconditions.empty()) {
        //       route_table[d_robot_id][timestep[d_robot_id] + 1]->is_satisfied = true;
        //       agent_cost = (route_table[d_robot_id][prev_step[d_robot_id]]->end.head(2) - route_table[d_robot_id][timestep[d_robot_id] + 1]->end.head(2)).norm();
        //       timestep[d_robot_id]++;
        //       cost_extend += agent_cost;
        //       costs[d_robot_id] = agent_cost;
        //       pass = true;
        //     }

        //     else{
        //       bool ddelay = false;
        //       for(int dp = 0; dp < route_table[d_robot_id][timestep[d_robot_id]+1]->preconditions.size(); dp++){
        //         auto d_precondirion = route_table[d_robot_id][timestep[d_robot_id]+1]->preconditions[dp];
        //         if(!d_precondirion.first) continue;
        //         int dd_robot_id = d_precondirion.first->robot_id;
        //         int dd_timestep = d_precondirion.first->timestep;

        //         if(dd_timestep > timestep[dd_robot_id] && precondition.second){
        //           ddelay = true;
        //           timestep[d_robot_id] = prev_step[d_robot_id];
        //           break;
        //         }
        //       }
        //       if(!ddelay){
        //         route_table[d_robot_id][timestep[d_robot_id] + 1]->is_satisfied = true;
        //         agent_cost = (route_table[d_robot_id][prev_step[d_robot_id]]->end.head(2) - route_table[d_robot_id][timestep[d_robot_id] + 1]->end.head(2)).norm();
        //         timestep[d_robot_id]++;
        //         cost_extend += agent_cost;
        //         costs[d_robot_id] = agent_cost;
        //         pass = true;
        //       }
        //     }
        //   }
        // }


        // if(pass) continue;

        if (d_timestep > timestep[d_robot_id] && precondition.second) {
        // if (!route_table[d_robot_id][d_timestep]->is_satisfied && precondition.second) {
          delay = true;
          timestep[i] = prev_step[i];
          if(costs[d_robot_id] != 0.0){
            agent_cost = costs[d_robot_id];
          }
          else{
            std::pair<int,int> delay_v = get_delay(d_robot_id, d_timestep, costs, route_table, timestep);
            if (delay_v.first < 0) {
              step_complete = false;
              agent_cost = 0.0;
              costs[i] = agent_cost;
              break;
            }
            agent_cost = (route_table[delay_v.first][prev_step[delay_v.first]]->end.head(2) - route_table[delay_v.first][delay_v.second]->end.head(2)).norm();
          }

          cost_extend += agent_cost;
          costs[i] = agent_cost;
          step_complete = false;
          break;
        }
      }


      if (!delay) {
        route_table[i][timestep[i] + 1]->is_satisfied = true;
        agent_cost = (route_table[i][prev_step[i]]->end.head(2) - route_table[i][timestep[i] + 1]->end.head(2)).norm();
        timestep[i]++;
        cost_extend += agent_cost;
        costs[i] = agent_cost;
      }
    }
  }
  if(complete == route_table.size()){
    return {-1.0, step_complete};
  }

  return {cost_extend, step_complete};
}

std::pair<int,int> ESES::get_delay(int robot_id, int timestep_a, std::vector<double> costs, const Graph& route_table, const std::vector<int>& timestep) {
  std::pair<int,int> find = {robot_id, timestep_a};
  while (true){
    int wp_timestep = find.second+1;
    for(int t = find.second+1; t<route_table[find.first].size(); t++){
      if(route_table[find.first][t]->is_waypoint) break;
      wp_timestep++;
    }
    wp_timestep = std::min(wp_timestep, (int)route_table[find.first].size()-1);
    if(route_table[find.first][wp_timestep]->preconditions.empty()){
      find = {find.first, wp_timestep};
      return find;
    }
    else{
      bool delay = false;
      for(int i = 0; i < route_table[find.first][wp_timestep]->preconditions.size(); i++){
        auto precondition = route_table[find.first][wp_timestep]->preconditions[i];
        if(!precondition.first) continue;
        int d_robot_id = precondition.first->robot_id;
        int d_timestep = precondition.first->timestep;
        if(d_timestep > timestep[d_robot_id] && precondition.second){
          find = {precondition.first->robot_id, precondition.first->timestep};
          delay = true;
          break;
        }
      }
      if(!delay){
        find = {find.first, wp_timestep};
        return find;
      }
      if(find.first == robot_id){
        std::clog << "Detected a dependency cycle while resolving ESES delays." << std::endl;
        return {-1, -1};
      }
    }
  }

  return find;
}


ESES::heuristic_return ESES::heuristic(Graph& route_table, std::vector<int> x){

  std::vector<int> x_update = x;

  init(route_table, x);

  bool g_check = true; // check update gcost(h_value count)

  while(true){
    auto step_sol = step(route_table, x);
    double cost_update = step_sol.first;
    if(cost_update == -1){
      break;
    }
    // if(cost_update < route_table.size()){
    //   g_check = false;
    // }

    if(!step_sol.second){
      g_check = false;
    }

    if(g_check){
      for(int t = 0; t < x.size(); t++){
        if(route_table[t].empty()) continue;
        x_update[t]++;
        // x[t]++;
      }
    }
    cost += cost_update;
  }
  // return heuristic_return{cost, x};
  return heuristic_return{cost, x_update};
}

ESES::branch_return ESES::branch(Graph& route_table, std::vector<int> x) {
  init(route_table, x);

  std::vector<int> path_length(route_table.size(), 0);
  for (int i = 0; i < route_table.size(); i++) {
    path_length[i] = static_cast<int>(route_table[i].size());
  }

  while (true) {
    for(int i = 0; i < route_table.size(); i++) {
      if(x[i] < path_length[i] && route_table[i][x[i]]->preconditions.size() > 0) {
        auto& preconditions = route_table[i][x[i]]->preconditions;
        for (auto precondition_it = preconditions.begin(); precondition_it != preconditions.end(); ++precondition_it) {
          if(!precondition_it->second) {
            if (!precondition_it->first) {
              preconditions.erase(precondition_it);
              break;
            }

            VertexRT* dependency = precondition_it->first;
            preconditions.erase(precondition_it);
            return branch_return{x, cost, {i, x[i], dependency->robot_id, dependency->timestep}};
          }
        }
      }
    }

    double cost_update = step(route_table, x).first;
    if(cost_update == -1.0){ break; }
    cost += cost_update;
  }

  return branch_return{x, cost, {}};
}

std::vector<std::pair<int, int>> ESES::getSuccessors(const Graph& curr_route_table, int robot_id, int timestep) {
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
              if (precondition.first->robot_id == robot_id && precondition.first->timestep == timestep) {
                  successors.emplace_back(i, j);
                  break;  // Prevent duplicate addition
              }
          }
      }
  }
  return successors;
}

bool ESES::is_cyclic(const Graph& curr_route_table, int start_robot_id, int start_timestep, int target_robot_id, int target_timestep) {
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

std::pair<Graph, int> ESES::runESES() {
  std::vector<int> x = curr_steps;
  auto hr_root = heuristic(ESES_graph, x);
  OpenList queue = OpenList();

  queue.push(new QueueNode{ESES_graph, x, 0, hr_root.cost, hr_root.cost});
  while(!queue.empty()) {
    auto parent_node = queue.top();
    queue.pop();

    auto br_node = branch(parent_node->route_table, parent_node->x);
    if(br_node.edge.empty()){
      return {parent_node->route_table, parent_node->f};
    }

    // Graph fix_graph = parent_node->route_table;

    Graph fix_graph = copy_graph(parent_node->route_table);
    fix_graph[br_node.edge[0]][br_node.edge[1]]->preconditions.push_back({fix_graph[br_node.edge[2]][br_node.edge[3]], true});
    if(!is_cyclic(fix_graph, br_node.edge[2], br_node.edge[3], br_node.edge[0], br_node.edge[1]))
    {
      auto hr_fix = heuristic(fix_graph, br_node.x);
      queue.push(new QueueNode{fix_graph, hr_fix.x, parent_node->g + cost, hr_fix.cost, ((parent_node->g + cost) + hr_fix.cost)});
    }

    // Graph reverse_graph = parent_node->route_table;

    Graph reverse_graph = copy_graph(parent_node->route_table);

    std::pair<int, int> swt_nodes = find_switchable_nodeds(br_node.edge[0], br_node.edge[1], br_node.edge[2], br_node.edge[3], reverse_graph);
    reverse_graph[br_node.edge[2]][swt_nodes.first]->preconditions.push_back({reverse_graph[br_node.edge[0]][swt_nodes.second], true});
    // reverse_graph[br_node.edge[0]][br_node.edge[1]]->preconditions.push_back({reverse_graph[br_node.edge[2]][br_node.edge[2]], true});

    if(!is_cyclic(reverse_graph, br_node.edge[0], swt_nodes.second, br_node.edge[2], swt_nodes.first))
    {
      auto hr_reverse = heuristic(reverse_graph, br_node.x);
      queue.push(new QueueNode{reverse_graph, hr_reverse.x, parent_node->g + cost, hr_reverse.cost, ((parent_node->g + cost) + hr_reverse.cost)});
    }

    // TODO: Check the switchable nodes
    // reverse_graph[br_node.edge[2]][br_node.edge[3]-1]->preconditions.push_back({reverse_graph[br_node.edge[0]][br_node.edge[1]+1], true});
    // if(!is_cyclic(reverse_graph, br_node.edge[2], br_node.edge[3], br_node.edge[0], br_node.edge[1]))
    // {
    //   auto hr_reverse = heuristic(reverse_graph, br_node.x);
    //   queue.push(new QueueNode{reverse_graph, hr_reverse.x, parent_node->g + cost, hr_reverse.cost, ((parent_node->g + cost) + hr_reverse.cost)});
    // }
  }

  return {ESES_graph, -1};
}

std::pair<int, int> ESES::find_switchable_nodeds(int robot_id, int timestep, int d_robot_id, int d_timestep, const Graph& route_table) {

  int switchable_node = -1;
  int d_switchable_node = -1;

  if(route_table[robot_id][timestep]->id_end == route_table[d_robot_id][d_timestep]->id_end) {
    return {d_timestep, timestep};
  }
  else if(d_timestep <= timestep) {
    for(int j = d_timestep-1; j >= 0; j--){
      if(route_table[d_robot_id][j]->is_waypoint){
        d_switchable_node = j;
        break;
      }
    }
    if(d_switchable_node == -1) d_switchable_node = 0;
    for(int i = timestep+1; i < route_table[robot_id].size(); i++){
      if(route_table[robot_id][i]->is_waypoint){
        switchable_node = i;
        break;
      }
    }
    if(switchable_node == -1) switchable_node = route_table[robot_id].size() - 1;
  }

  else if(d_timestep > timestep) {
    for(int j = d_timestep+1; j < route_table[d_robot_id].size(); j++){
      if(route_table[d_robot_id][j]->is_waypoint){
        d_switchable_node = j;
        break;
      }
      if(d_switchable_node == -1) d_switchable_node = route_table[d_robot_id].size() - 1;
    }
    for(int i = timestep-1; i >= 0; i--){
      if(route_table[robot_id][i]->is_waypoint){
        switchable_node = i;
        break;
      }
    }
    if(switchable_node == -1) switchable_node = 0;
  }
  return {d_switchable_node, switchable_node};
}

double ESES::get_costs(){
  init(ESES_graph, curr_steps);

  bool g_check = true; // check update gcost(h_value count)

  while(true){
    auto step_sol = step(ESES_graph, curr_steps);
    double cost_update = step_sol.first;
    if(cost_update == -1){
      break;
    }

    // Debugging output
    // std::cout << "Cost update: " << cost_update << std::endl;
    // std::cout << "Current steps: ";
    // for (int i = 0; i<curr_steps.size(); i++) {
    //   std::cout << curr_steps[i] << " (" << ESES_graph[i].size()-1 << ") ";
    // }
    // std::cout << std::endl;

    cost += cost_update;
  }

  return cost;
}

Graph ESES::copy_graph(const Graph& original_graph) {
  Graph copied_graph;

  // Resize the copied graph to match the original graph
  copied_graph.resize(original_graph.size());

  for (size_t i = 0; i < original_graph.size(); ++i) {
      copied_graph[i].resize(original_graph[i].size());
      for (size_t j = 0; j < original_graph[i].size(); ++j) {
          if (original_graph[i][j] != nullptr) {
              // Create a new Node object by copying the original
              VertexRT* new_node = new VertexRT(*original_graph[i][j]);

              // Deep copy the preconditions
              new_node->preconditions.clear();
              for (const auto& precondition : original_graph[i][j]->preconditions) {
                  if (precondition.first != nullptr) {
                      new_node->preconditions.push_back({new VertexRT(*precondition.first), precondition.second});
                  } else {
                      new_node->preconditions.push_back({nullptr, false});
                  }
              }

              // Assign the new node to the copied graph
              copied_graph[i][j] = new_node;
          } else {
              copied_graph[i][j] = nullptr;
          }
      }
  }

  return copied_graph;
}


}
