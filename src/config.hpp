#pragma once

#include <string>
#include <vector>

#include "graphsearch/graph.hpp"
#include "graphsearch/heuristics.hpp"
#include "graphsearch/search.hpp"

namespace graphsearch {

struct AlgorithmSpec {
    std::string type;       ///< Dijkstra, AStar, WeightedAStar, Greedy, BFS, BellmanFord, Bidirectional
    std::string heuristic;  ///< "", zero, euclidean, manhattan, octile (informed algorithms)
    double weight = 1.0;    ///< WeightedAStar only
};

struct Problem {
    Graph<double> graph;
    NodeId start = 0;
    NodeId goal = 0;
    Points coords;  ///< null when the YAML has no `nodes:` section
    std::vector<AlgorithmSpec> algorithms;
};

/// Loads a problem from YAML (see config/graph.yaml). Throws on malformed input.
Problem loadProblem(const std::string& path);

/// Runs one configured algorithm. Throws std::invalid_argument on unknown type/heuristic.
SearchResult<double> run(const Problem& p, const AlgorithmSpec& spec);

}  // namespace graphsearch
