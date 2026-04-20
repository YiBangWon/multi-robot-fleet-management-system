// Single-agent path planner declarations used by the ECBS stack.

#pragma once
#include "lowlevel_node.h"

class PathPlanner
{
public:

    PathPlanner(): min_f(0) {};
    ~PathPlanner(){};

    Path Run(const SolverMap& map, const State& start, const std::vector<std::pair<int, int>>& goals);
    void Release();
    std::vector<double> getHeuristics(const SolverMap& solver_map, int loc);
    double getH(const SolverMap& solver_map, int curr, int gid, const std::vector<std::pair<int, int>>& goals) const;
    void setCTs(const std::list<CT>& cts_);
    bool checkCTs(int curr_loc, int next_loc, int next_t);
    Path getFinalPath(const NodeLL* goal);
	int getEdgeIndex(int from, int to) const {return (from + 1) * num_nodes + to; }
	std::pair<int, int> getEdge(int index) const {return std::make_pair(index / num_nodes - 1, index % num_nodes); }

    double path_cost;
    double min_f;
    size_t num_nodes;
    int window;

    boost::unordered_map<size_t, std::list<std::pair<int, int>>> cts;        // [loc/edge] -> [time_min, time_max]
    boost::unordered_map<size_t, std::list<std::pair<int, int>>> init_cts;   // [loc/edge] -> [time_min, time_max]
    
	boost::heap::fibonacci_heap<NodeLL*, boost::heap::compare<NodeLL::compare_node>> OPEN;
	boost::unordered_set<NodeLL*, NodeLL::Hasher, NodeLL::equal_node> all_nodes;
	
};
