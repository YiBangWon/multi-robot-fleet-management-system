// Solver-map structures and heuristic caches used by the planner.

#include <eigen3/Eigen/Dense>
#include <vector>
#include "constraint.h"
#define WEIGHT_MAX 99999

struct State
{
    State(): loc(-1), t(-1), dir(-1) {}
    State(int loc, int t = -1, int dir = -1):loc(loc), t(t), dir(dir) {}
    State(const State& other) {loc = other.loc; t = other.t; dir = other.dir;}
    State wait() const {return State(loc, t + 1, dir);}
    struct Hasher{
        std::size_t operator()(const State& n) const{
            size_t loc_hash = std::hash<int>()(n.loc);
            size_t time_hash = std::hash<int>()(n.t);
            size_t ori_hash = std::hash<int>()(n.dir);
            return (time_hash ^ (loc_hash << 1) ^ (ori_hash << 2));
        }
    };
    void operator = (const State& other){
        t = other.t;    loc = other.loc;    dir = other.dir;
    }
    bool operator == (const State& other) const{
        return t == other.t && loc == other.loc && dir == other.dir;
    }
    bool operator != (const State& other) const{
        return t != other.t || loc != other.loc || dir != other.dir;
    }
    int loc;
    int t;
    int dir;
};
typedef std::vector<State> Path;        // Path

class SolverMap
{
public:

    SolverMap(){};
    SolverMap(std::vector<Eigen::Vector2d> locations,   std::vector<std::vector<int>> neighbors, 
              std::vector<std::vector<int>> rneighbors, std::vector<std::vector<double>> weights)
              : locations(locations), neighbors(neighbors), rneighbors(rneighbors), weights(weights){};

    std::list<std::pair<State, double>> getNBR(const State& s) const;
    std::list<std::pair<State, double>> getReverseNBR(const State& s) const;

    bool rotation;
    int num_nodes;
    std::vector<Eigen::Vector2d> locations;
    std::vector<std::vector<int>> neighbors;
    std::vector<std::vector<int>> rneighbors;
    std::vector<std::vector<double>> weights;
    boost::unordered_map<int, std::vector<double>> heuristics;

};