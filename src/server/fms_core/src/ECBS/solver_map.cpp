// Solver-map precomputation and heuristic caching for repeated planning queries.

#include <ECBS/solver_map.h>

std::list<std::pair<State, double>> SolverMap::getNBR(const State& s) const{
    std::list<std::pair<State, double>> nbr;
    if (s.loc < 0) return nbr;

    if (rotation){
        nbr.push_back( std::make_pair(State(s.loc, s.t + 1, s.dir), weights[s.loc][4]) ); // wait
        if (weights[s.loc][s.dir] < WEIGHT_MAX - 1)
            nbr.push_back(std::make_pair(State(neighbors[s.loc][s.dir], s.t + 1, s.dir), weights[s.loc][s.dir])); // move
        int next_dir1 = s.dir + 1;
        int next_dir2 = s.dir - 1;
        if (next_dir2 < 0)      next_dir2 += 4;
        else if (next_dir1 > 3) next_dir1 -= 4;
        nbr.push_back(std::make_pair(State(s.loc, s.t + 1, next_dir1), weights[s.loc][4])); // turn left
        nbr.push_back(std::make_pair(State(s.loc, s.t + 1, next_dir2), weights[s.loc][4])); // turn right
    }
    else {
        nbr.push_back( std::make_pair(State(s.loc, s.t + 1, 0), weights[s.loc][4]) ); // wait
        for (int i = 0; i < 4; i++){ // move
            if (weights[s.loc][i] < WEIGHT_MAX - 1)
                nbr.push_back(std::make_pair(State(neighbors[s.loc][i], s.t + 1, 0), weights[s.loc][i])); // move
        }
    }
    return nbr;
}


std::list<std::pair<State, double>> SolverMap::getReverseNBR(const State& s) const{

    std::list<std::pair<State, double>> rnbr;    
    if (rotation){
        int rloc = rneighbors[s.loc][s.dir];
        if (rloc >= 0)
            rnbr.push_back( std::make_pair(State(rloc, -1, s.dir), weights[rloc][s.dir])); // move
        int next_dir1 = s.dir + 1;
        int next_dir2 = s.dir - 1;
        if (next_dir2 < 0)      next_dir2 += 4;
        else if (next_dir1 > 3) next_dir1 -= 4;
        rnbr.push_back(std::make_pair(State(s.loc, -1, next_dir1), weights[s.loc][4])); // turn right
        rnbr.push_back(std::make_pair(State(s.loc, -1, next_dir2), weights[s.loc][4])); // turn left
    }
    else {
        for (int i = 0; i < 4; i++){ // move
            int rloc = rneighbors[s.loc][i];
            if (rloc >= 0)
                rnbr.push_back( std::make_pair(State(rloc, -1, 0), weights[rloc][i])); // move
        }
    }    
    return rnbr;
}
