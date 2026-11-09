#pragma once

#include <string>
#include <vector>

#include "graph.hpp"

namespace graphsearch {

struct AlgorithmSpec {
    std::string type;  ///< "Dijkstra" or "AStar"
};

struct Problem {
    Graph<double> graph;
    NodeId start = 0;
    NodeId goal = 0;
    std::vector<AlgorithmSpec> algorithms;
};

/// Loads a problem from YAML (see config/graph.yaml). Throws on malformed input.
Problem loadProblem(const std::string& path);

}  // namespace graphsearch
