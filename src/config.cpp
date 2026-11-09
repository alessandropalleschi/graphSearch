#include "graphsearch/config.hpp"

#include <yaml-cpp/yaml.h>

namespace graphsearch {

Problem loadProblem(const std::string& path) {
    const YAML::Node cfg = YAML::LoadFile(path);
    Problem p;
    p.graph = Graph<double>(cfg["totalNodes"].as<std::size_t>());
    p.start = cfg["startNode"].as<NodeId>();
    p.goal = cfg["goalNode"].as<NodeId>();
    for (const auto& e : cfg["edges"]) {
        p.graph.addEdge(e["source"].as<NodeId>(), e["destination"].as<NodeId>(),
                        e["weight"].as<double>());
    }
    for (const auto& a : cfg["searchAlgorithms"]) {
        p.algorithms.push_back({a["type"].as<std::string>()});
    }
    return p;
}

}  // namespace graphsearch
