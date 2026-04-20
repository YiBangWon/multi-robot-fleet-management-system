// Constraint definitions shared by the ECBS solver.

#pragma once
#include <tuple>
#include <list>
#include <vector>
#include <iostream>
#include <ctime>
#include <fstream>
#include <boost/heap/fibonacci_heap.hpp>
#include <boost/unordered_set.hpp>
#include <boost/unordered_map.hpp>
typedef std::tuple<int, int, int, int> CT;           // Constraint
typedef std::tuple<int, int, int, int, int> CF;      // Conflict
