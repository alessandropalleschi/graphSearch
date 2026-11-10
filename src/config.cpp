#include "config.hpp"

#include <yaml-cpp/yaml.h>

#include <memory>

namespace graphsearch {

Problem loadProblem(const std::string& path) {
    const YAML::Node cfg = YAML::LoadFile(path);
    Problem p;
    const auto n = cfg["totalNodes"].as<std::size_t>();
    p.graph = Graph<double>(n);
    p.start = cfg["startNode"].as<NodeId>();
    p.goal = cfg["goalNode"].as<NodeId>();

    if (cfg["nodes"]) {
        auto pts = std::make_shared<std::vector<Point>>(n);
        for (const auto& node : cfg["nodes"]) {
            const auto id = node["id"].as<NodeId>();
            if (id >= n) throw std::out_of_range("node id out of range in `nodes`");
            (*pts)[id] = {node["x"].as<double>(), node["y"].as<double>()};
        }
        p.coords = pts;
    }
    for (const auto& e : cfg["edges"]) {
        p.graph.addEdge(e["source"].as<NodeId>(), e["destination"].as<NodeId>(),
                        e["weight"].as<double>());
    }
    p.graph.finalize();

    for (const auto& a : cfg["searchAlgorithms"]) {
        AlgorithmSpec s;
        s.type = a["type"].as<std::string>();
        if (a["heuristic"]) s.heuristic = a["heuristic"].as<std::string>();
        if (a["weight"]) s.weight = a["weight"].as<double>();
        p.algorithms.push_back(std::move(s));
    }
    return p;
}

static Heuristic<double> makeHeuristic(const Problem& p, const std::string& name) {
    if (name == "zero" || (name.empty() && !p.coords)) return zeroHeuristic<double>();
    const std::string h = name.empty() ? "euclidean" : name;
    if (!p.coords) throw std::invalid_argument("heuristic '" + h + "' needs a `nodes:` section");
    if (h == "euclidean") return euclidean<double>(p.coords);
    if (h == "manhattan") return manhattan<double>(p.coords);
    if (h == "octile") return octile<double>(p.coords);
    throw std::invalid_argument("unknown heuristic: " + h);
}

SearchResult<double> run(const Problem& p, const AlgorithmSpec& s, std::vector<NodeId>* trace) {
    const auto& g = p.graph;
    if (s.type == "Dijkstra") return dijkstra(g, p.start, p.goal, trace);
    if (s.type == "BFS") return bfs(g, p.start, p.goal, trace);
    if (s.type == "BellmanFord") return bellmanFord(g, p.start, p.goal);
    if (s.type == "Bidirectional") return bidirectionalDijkstra(g, p.start, p.goal, trace);
    if (s.type == "AStar") return astar(g, p.start, p.goal, makeHeuristic(p, s.heuristic), trace);
    if (s.type == "WeightedAStar")
        return weightedAstar(g, p.start, p.goal, makeHeuristic(p, s.heuristic), s.weight, trace);
    if (s.type == "Greedy")
        return greedyBestFirst(g, p.start, p.goal, makeHeuristic(p, s.heuristic), trace);
    throw std::invalid_argument("unknown search algorithm: " + s.type);
}

}  // namespace graphsearch
