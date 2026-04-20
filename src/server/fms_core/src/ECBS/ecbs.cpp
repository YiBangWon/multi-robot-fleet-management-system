// Implementation of the ECBS search routine used for collision-aware multi-agent pathfinding.

#include <ECBS/ecbs.h>

ECBS::ECBS(const SolverMap& solver_map, PathPlanner& path_planner): 
        solver_map(solver_map), path_planner(path_planner){}
        
ECBS::~ECBS(){ Release(); }

void ECBS::Clear(){
    count = 0;
    min_f = -1;
    focal_thr = -1;
    paths.clear();
    path_min_costs.clear();
    path_costs.clear();
    OPEN.clear();
    FOCAL.clear();
    starts.clear();
    goals.clear();
    Release();
}

bool ECBS::Run(const std::vector<State>& starts_, const std::vector<std::vector<std::pair<int,int>>>& goals_, int time_limit){

    Clear();
    
    starts = starts_;
    goals = goals_;
    num_of_robots = starts.size();
    bool solution_found = false;
    double current_time = 0;
    NodeHL* solution_node;

    if (!genRoot())
        return false;

    std::clock_t start_time = std::clock();
    while (!OPEN.empty()){

        count++;

        current_time = (std::clock() - start_time)*1.0/CLOCKS_PER_SEC;
        if (current_time > time_limit)  break;

        NodeHL* node = Pop();
        setPaths(node);

        if (node->cfs.empty()){
			solution_found = true;
            solution_node = node;
            getSolution(node);
			break;
        }
        selectCF(*node);

        NodeHL* child_node_left = new NodeHL(node);
        NodeHL* child_node_right = new NodeHL(node);
        resolveCF(*node->select_cf, child_node_left, child_node_right);
        
        std::vector<Path*> curr_paths(paths);
        bool found1 = genChild(child_node_left, node);
        if (found1){ Push(child_node_left); }
        else       { delete(child_node_left); child_node_left = nullptr; }
        paths = curr_paths;

        bool found2 = genChild(child_node_right, node);
        if (found2){ Push(child_node_right); }
        else       { delete(child_node_right); child_node_right = nullptr; }
        paths = curr_paths;

    }

    if (solution_found && !validateSolution()){
        return false;
    }

    if (!solution_found){
        std::cout << "Solution not found!! #" << count << " iterations" << std::endl;
    }
    else {
        // std::cout << "count:" << count << ", min_f: " << solution_node->min_f << ", g: " << solution_node->g << ", f: " << solution_node->f << std::endl;
    }

    return solution_found;
}


bool ECBS::genRoot(){
    paths.resize(num_of_robots, nullptr);
    path_min_costs.resize(num_of_robots, 0);
    path_costs.resize(num_of_robots, 0);
    root_node = new NodeHL();
    root_node->d = 0;
    for (int i = 0; i < num_of_robots; i++){        
        path_planner.setCTs(std::list<CT>());
        Path path = path_planner.Run(solver_map, starts[i], goals[i]);
		if (path.empty())
            return false;
        root_node->paths.emplace_back(i, path, path_planner.min_f, path_planner.path_cost);
        paths[i] = &std::get<1>(root_node->paths.back());
        path_min_costs[i] = path_planner.min_f;
        path_costs[i] = path_planner.path_cost;
        root_node->g += path_planner.path_cost;
        root_node->min_f += path_planner.min_f;
    }
    findCFs(root_node->cfs);
    root_node->f = root_node->g;
    root_node->num_conf = root_node->cfs.size();
    min_f = root_node->min_f;
    focal_thr = min_f * suboptimal_bound;
    Push(root_node);
    return true;
}


bool ECBS::genChild(NodeHL* node, NodeHL* parent){

    int a, v1, v2, t;
    std::tie(a, v1, v2, t) = node->cts.front();
    if (!findSinglePath(node, a))
        return false;
    copyCFs(node->parent->cfs, node->cfs, a);
    findCFs(node->cfs, a);
    node->num_conf = node->cfs.size();
    node->f = node->g;
    return true;
}


void ECBS::Push(NodeHL* node){
    node->open_handle = OPEN.push(node);
    if (node->f <= focal_thr){
        node->focal_handle = FOCAL.push(node);
    }
    all_nodes.push_back(node);
}


NodeHL* ECBS::Pop(){
    updateFOCAL();
    NodeHL* node = FOCAL.top();
    FOCAL.pop();
    OPEN.erase(node->open_handle);
    return node;
}


void ECBS::setPaths(NodeHL* node){
    std::vector<bool> is_updated(num_of_robots, false);
    while (node != nullptr){
        for (auto p = node->paths.begin(); p != node->paths.end(); ++p){
            int a = std::get<0>(*p);
            if (!is_updated[a]){
                paths[a] = &(std::get<1>(*p));
                path_min_costs[a] = std::get<2>(*p);
                path_costs[a] = std::get<3>(*p);
                is_updated[a] = true;
            }
        }
        node = node->parent;
    }
}


bool ECBS::findSinglePath(NodeHL* node, int a){
    std::list<CT> cts;
    NodeHL* curr_node = node;
    while (curr_node != root_node){
        for (CT c : curr_node->cts){
            if (std::get<0>(c) == a){
                cts.push_back(c);
            }
        }
        curr_node = curr_node->parent;
    }
    path_planner.setCTs(cts);
    Path path = path_planner.Run(solver_map, starts[a], goals[a]);
    if (path.empty())   return false;
    node->g = node->g - path_costs[a] + path_planner.path_cost;
    node->min_f = node->min_f - path_min_costs[a] + path_planner.min_f;
    for (auto it = node->paths.begin(); it != node->paths.end(); ++it){
        if (std::get<0>(*it) == a){
            node->paths.erase(it);
            break;
        }
    }
    node->paths.emplace_back(a, path, path_planner.min_f, path_planner.path_cost);
    paths[a] = &std::get<1>(node->paths.back());
    return true;
}


void ECBS::selectCF(NodeHL &node) const{
    if (node.cfs.empty()) return;
    node.select_cf = node.cfs.front();
    for (auto cf : node.cfs){

        if (std::get<4>(*cf) < std::get<4>(*node.select_cf)){
            node.select_cf = cf;
        }
        else if (std::get<4>(*cf) == std::get<4>(*node.select_cf)){
            if (rand()%2 == 0){
                node.select_cf = cf;
            }
        }
    }
}


void ECBS::resolveCF(const CF& cf, NodeHL* n1, NodeHL* n2){
    int a1, a2, v1, v2, t;
    std::tie(a1, a2, v1, v2, t) = cf;
    if (v2 < 0){                                    // 1) Vertex Conflict
        n1->cts.emplace_back(a1, v1, v2, t);
        n2->cts.emplace_back(a2, v1, v2, t);
    }
    else {                                          // 2) Edge Conflict
        n1->cts.emplace_back(a1, v1, v2, t);
        n2->cts.emplace_back(a2, v2, v1, t);
    }
}


void ECBS::copyCFs(const std::list<std::shared_ptr<CF>>& cfs, std::list<std::shared_ptr<CF> >& copy, int a) const{
    for (auto it = cfs.begin(); it != cfs.end(); ++it){
        if (a == std::get<0>(**it) || a == std::get<1>(**it)){
            // Do nothing
        }
        else {
            copy.push_back(*it);
        }
    }
}


void ECBS::findCFs(std::list<std::shared_ptr<CF>>& cfs) const{
    for (int a1 = 0; a1 < num_of_robots; a1++){
        for (int a2 = a1 + 1; a2 < num_of_robots; a2++){
            findCFs(cfs, a1, a2);
        }
    }
}


void ECBS::findCFs(std::list<std::shared_ptr<CF>>& cfs, int a1) const{
    for (int a2 = 0; a2 < num_of_robots; a2++){
        if(a1 == a2) continue;
        findCFs(cfs, a1, a2);
    }
}


void ECBS::findCFs(std::list<std::shared_ptr<CF>>& cfs, int a1, int a2) const{
    if (paths[a1] == nullptr || paths[a2] == nullptr)   return;
    int size1 = std::min(window + 1, (int)paths[a1]->size());
    int size2 = std::min(window + 1, (int)paths[a2]->size());
    for (int t = 0; t < size1; t++){
        if (size2 <= t) break;
        int loc1 = paths[a1]->at(t).loc;
        int loc2 = paths[a2]->at(t).loc;
        if (loc1 == loc2){
            cfs.emplace_back(new CF(a1, a2, loc1, -1, t));
            return;
        }
        else if (t < size1 - 1 && t < size2 - 1
            && loc1 == paths[a2]->at(t + 1).loc
            && loc2 == paths[a1]->at(t + 1).loc){
            cfs.emplace_back(new CF(a1, a2, loc1, loc2, t + 1)); // edge conflict
            return;
        }
    }
}


void ECBS::updateFOCAL(){
    NodeHL* open_top = OPEN.top();
    if (open_top->min_f > min_f){        
        min_f = open_top->min_f;
        double new_focal_thr = min_f * suboptimal_bound;
        for (NodeHL* n : OPEN){
            if (n->f > focal_thr && n->f <= new_focal_thr){
                n->focal_handle = FOCAL.push(n);
            }
        }
        focal_thr = new_focal_thr;
    }
}


void ECBS::Release(){
    for (auto it = all_nodes.begin(); it != all_nodes.end(); it++)
        delete *it;
    all_nodes.clear();
}


void ECBS::getSolution(NodeHL* node){
    setPaths(node);
    solution.resize(num_of_robots);
    for (int a = 0; a < num_of_robots; a++){
        solution[a] = *paths[a];
    }
}


bool ECBS::validateSolution() const{
    std::list<std::shared_ptr<CF>> conflict;
    for (int a1 = 0; a1 < num_of_robots; a1++){
        for (int a2 = a1 + 1; a2 < num_of_robots; a2++){
            findCFs(conflict, a1, a2);
            if (!conflict.empty()){
                return false;
            }
        }
    }
    return true;
}
