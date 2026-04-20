// Low-level ECBS search-node definition.

#include "solver_map.h"

class NodeLL
{
public:
    State state;
    NodeLL* parent;
    double g;
    double h;
    bool inopen;
    int gid;

    double f() const { return g + h; }

    NodeLL(): g(0), h(0), parent(nullptr), inopen(false), gid(0) {}
    NodeLL(const State& state, double g, double h, NodeLL* parent):
        state(state), g(g), h(h), parent(parent), inopen(false){
        if (parent != nullptr){gid = parent->gid;}
        else {gid = 0;}
    }
    
    struct compare_node{
        bool operator()(const NodeLL* n1, const NodeLL* n2) const {
            return n1->g + n1->h >= n2->g + n2->h;
        }
    };

    struct equal_node {
        bool operator() (const NodeLL* n1, const NodeLL* n2) const{
            return (n1 == n2) || (n1 && n2 && n1->state == n2->state && n1->gid == n2->gid);
        }
    };

    struct Hasher{
        std::size_t operator()(const NodeLL* n) const{return State::Hasher()(n->state);}
    };
    
    boost::heap::fibonacci_heap<NodeLL*, boost::heap::compare<NodeLL::compare_node>>::handle_type open_handle;
};

