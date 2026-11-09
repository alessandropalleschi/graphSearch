#include <iostream>

#include "config.hpp"

using namespace graphsearch;

static void print(const AlgorithmSpec& s, const SearchResult<double>& r) {
    std::cout << s.type;
    if (!s.heuristic.empty()) std::cout << '(' << s.heuristic << ')';
    std::cout << ": ";
    if (r.negativeCycle) {
        std::cout << "negative cycle reachable from start\n";
    } else if (!r.found) {
        std::cout << "no path found\n";
    } else {
        std::cout << "cost=" << r.cost << " expanded=" << r.expanded << " path=";
        for (NodeId n : r.path) std::cout << n << ' ';
        std::cout << '\n';
    }
}

int main(int argc, char** argv) {
    const std::string arg = argc > 1 ? argv[1] : "config/graph.yaml";
    if (arg == "-h" || arg == "--help") {
        std::cout << "usage: graphSearch [config.yaml]   (default: config/graph.yaml)\n";
        return 0;
    }
    try {
        const Problem p = loadProblem(arg);
        for (const auto& spec : p.algorithms) print(spec, run(p, spec));
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << '\n';
        return 1;
    }
}
