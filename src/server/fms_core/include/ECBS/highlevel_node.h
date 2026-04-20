// High-level ECBS search-node definition.

#pragma once
#include "constraint.h"

class NodeHL
{
public:
    NodeHL(): parent(nullptr), g(0), min_f(0) {}
    NodeHL(NodeHL* parent): parent(parent){
        g = parent->g;
        min_f = parent->min_f;
        d = parent->d + 1;
    }
    ~NodeHL(){};
    void clear(){cfs.clear();}

    struct compare_OPEN{
        bool operator()(const NodeHL* n1, const NodeHL* n2) const{
            return n1->min_f >= n2->min_f;
        }
    };
    struct compare_FOCAL{
        bool operator()(const NodeHL* n1, const NodeHL* n2) const{
            if (n1->num_conf == n2->num_conf){
                return n1->f >= n2->f;
            }
     		return n1->num_conf >= n2->num_conf;
     	}
    };
    typedef boost::heap::fibonacci_heap<NodeHL*,boost::heap::compare<NodeHL::compare_OPEN>>::handle_type openHandle;
    typedef boost::heap::fibonacci_heap<NodeHL*,boost::heap::compare<NodeHL::compare_FOCAL>>::handle_type focalHandle;
    openHandle open_handle;
    focalHandle focal_handle;

    std::shared_ptr<CF> select_cf;
    std::list<std::shared_ptr<CF> > cfs;
    std::list<CT> cts;

    NodeHL* parent;
    std::list<std::tuple<int, Path, double, double>> paths; // <agent_id, path, lower_bound, path_cost>
    double g;
    double f;
    double min_f;
    int d;
    int num_conf;

};