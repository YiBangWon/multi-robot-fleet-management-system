// Core ECBS solver declarations.


#pragma once
#include "path_planner.h"
#include "highlevel_node.h"
#include <ctime>

class ECBS
{
public:

    ECBS(const SolverMap& solver_map, PathPlanner& path_planner);
    ~ECBS();

    bool Run(const std::vector<State>& starts_, const std::vector<std::vector<std::pair<int, int>>>& goals_, int time_limit);
    void Clear();
    inline void Release();
    
    bool genRoot();
    bool genChild(NodeHL* child, NodeHL* curr);

    void Push(NodeHL* node);
    NodeHL* Pop();

    void setPaths(NodeHL* node);
    bool findSinglePath(NodeHL* node, int a);
    void selectCF(NodeHL &parent) const;
    void resolveCF(const CF& cf, NodeHL* n1, NodeHL* n2);

    void findCFs(std::list<std::shared_ptr<CF> >& cfs) const;
    void findCFs(std::list<std::shared_ptr<CF> >& cfs, int a1) const;
    void findCFs(std::list<std::shared_ptr<CF> >& cfs, int a1, int a2) const;
    void copyCFs(const std::list<std::shared_ptr<CF>>& cfs, std::list<std::shared_ptr<CF> >& copy, int a) const;

    void updateFOCAL();
    void getSolution(NodeHL* node);
    bool validateSolution() const;

    std::vector<State> starts;
    std::vector<std::vector<std::pair<int, int>>> goals;
    int window;
	double suboptimal_bound;

    NodeHL* root_node;

    int num_of_robots;
    double min_f;
    double focal_thr;

	PathPlanner& path_planner;
	const SolverMap& solver_map;
    std::vector<Path> solution;

    typedef boost::heap::fibonacci_heap<NodeHL*,boost::heap::compare<NodeHL::compare_OPEN>> openHeap;
    typedef boost::heap::fibonacci_heap<NodeHL*,boost::heap::compare<NodeHL::compare_FOCAL>> focalHeap;
    openHeap OPEN;
    focalHeap FOCAL;
    std::list<NodeHL*> all_nodes;

    std::vector<Path*> paths;
    std::vector<double> path_min_costs;
    std::vector<double> path_costs;

    int count;

};
