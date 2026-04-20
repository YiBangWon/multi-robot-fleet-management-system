// Single-agent path planner and heuristic support used by the ECBS stack.

#include <ECBS/path_planner.h>


Path PathPlanner::Run(const SolverMap& solver_map, const State& start, const std::vector<std::pair<int, int> >& goals){

	double h = getH(solver_map, start.loc, 0, goals);
	if (h >= WEIGHT_MAX || checkCTs(start.loc, start.loc, 0)){
		return Path();
	}

	NodeLL* root_node;
    root_node = new NodeLL(start, 0, h, nullptr);
    root_node->open_handle = OPEN.push(root_node);
    root_node->inopen = true;
    min_f = 0;
    all_nodes.insert(root_node);

    while (!OPEN.empty()){

        NodeLL* node = OPEN.top(); OPEN.pop();
        node->inopen = false;
        min_f = node->f();
        
        if (node->state.loc == goals[node->gid].first && node->state.t >= goals[node->gid].second){
			node->gid++;
        }

		if (node->gid == (int)goals.size() || node->g > window){
			Path path = getFinalPath(node);
			Release();
			OPEN.clear();
			return path;
		}

        for (auto next_s: solver_map.getNBR(node->state)){
            if (!checkCTs(node->state.loc, next_s.first.loc, next_s.first.t)){
                double next_g = node->g + next_s.second;
                double next_h = getH(solver_map, next_s.first.loc, node->gid, goals);
                if (next_h >= WEIGHT_MAX)   continue;
                auto next_node = new NodeLL(next_s.first, next_g, next_h, node);
                auto it = all_nodes.find(next_node);
                if (it == all_nodes.end()){
                    next_node->inopen = true;
                    next_node->open_handle = OPEN.push(next_node);
                    all_nodes.insert(next_node);
                }
                else {
                    NodeLL* prev_node = *it;
                    if (prev_node->inopen){                        
                        if (prev_node->f() > next_g + next_h){
                            prev_node->g = next_g;
                            prev_node->h = next_h;
                            prev_node->parent = node;
                            OPEN.increase(prev_node->open_handle);
                        }
                    }
                    delete(next_node);
                }
            }
        } 
    }
    Release();
    OPEN.clear();
    return Path();
}


std::vector<double> PathPlanner::getHeuristics(const SolverMap& solver_map, int loc){

	boost::heap::fibonacci_heap<NodeLL*, boost::heap::compare<NodeLL::compare_node>> heap;
    boost::unordered_set<NodeLL*, NodeLL::Hasher, NodeLL::equal_node> nodes;

    State root_state(loc, -1, 0);
    for (auto neighbor : solver_map.getReverseNBR(root_state)){
        NodeLL* root = new NodeLL(neighbor.first, 0, 0, nullptr);
        root->open_handle = heap.push(root);
        nodes.insert(root);
    }
	while (!heap.empty()){
        NodeLL* curr = heap.top();  heap.pop();        
		for (auto next_s : solver_map.getReverseNBR(curr->state)){
			double next_g = curr->g + next_s.second;
            NodeLL* next_node = new NodeLL(next_s.first, next_g, 0, nullptr);
			auto it = nodes.find(next_node);
			if (it == nodes.end()) {
				next_node->open_handle = heap.push(next_node);
				nodes.insert(next_node);
			}
			else { 
				delete(next_node);
                NodeLL* prev_next = *it;
				if (prev_next->g > next_g){
					prev_next->g = next_g;
					heap.increase(prev_next->open_handle);
				}
			}
		}
	}
    std::vector<double> heuristics(solver_map.num_nodes, WEIGHT_MAX);
	for (auto it = nodes.begin(); it != nodes.end(); it++){
        NodeLL* s = *it;
		heuristics[s->state.loc] = std::min(s->g, heuristics[s->state.loc]);
		delete (s);
	}
	nodes.clear();
	heap.clear();
    return heuristics;
}

Path PathPlanner::getFinalPath(const NodeLL* goal){

    Path path(goal->state.t + 1);
    path_cost = goal->f();
    const NodeLL* curr = goal;
    for(int t = goal->state.t; t >= 0; t--){
        path[t] = curr->state;
        curr = curr->parent;
    }
    return path;
}


double PathPlanner::getH(const SolverMap& solver_map, int curr, int gid, const std::vector<std::pair<int, int> >& goals) const{
    double h = solver_map.heuristics.at(goals[gid].first)[curr];
    gid++;
    while (gid < (int) goals.size()){
        h += solver_map.heuristics.at(goals[gid].first)[goals[gid - 1].first];
        gid++;
    }
    return h;
}

void PathPlanner::Release(){
    for (auto n = all_nodes.begin(); n != all_nodes.end(); n++)
        delete (*n);
    all_nodes.clear();
}

bool PathPlanner::checkCTs(int curr_loc, int next_loc, int next_t){
    
	auto it = cts.find(next_loc);
	if (it != cts.end()){
		for (auto time_range : it->second){
			if (next_t >= time_range.first && next_t < time_range.second)
				return true;
		}
	}

	if (curr_loc != next_loc){
		it = cts.find(getEdgeIndex(curr_loc, next_loc));
		if (it != cts.end()){
			for (auto time_range : it->second){
				if (next_t >= time_range.first && next_t < time_range.second)
					return true;
			}
		}
	}
	return false;
}

void PathPlanner::setCTs(const std::list<CT>& cts_){
    cts.clear();
    cts = init_cts;
    for (auto con : cts_){
		if (std::get<2>(con) < 0)   // vertex constraint
			cts[std::get<1>(con)].emplace_back(std::get<3>(con), std::get<3>(con) + 1);
		else                        // edge constraint
			cts[getEdgeIndex(std::get<1>(con), std::get<2>(con))].emplace_back(std::get<3>(con), std::get<3>(con) + 1);
    }
}