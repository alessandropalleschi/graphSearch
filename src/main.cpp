#include <iostream>

#include "graphsearch/config.hpp"
#include "graphsearch/search.hpp"

using namespace graphsearch;

static void print(const char* name, const SearchResult<double>& r) {
    std::cout << name << ": ";
    if (!r.found) {
        std::cout << "no path found\n";
        return;
    }
    std::cout << "cost=" << r.cost << " expanded=" << r.expanded << " path=";
    for (NodeId n : r.path) std::cout << n << ' ';
    std::cout << '\n';
}

int main(int argc, char** argv) {
    const std::string file = argc > 1 ? argv[1] : "config/graph.yaml";
    try {
        const Problem p = loadProblem(file);
        for (const auto& a : p.algorithms) {
            if (a.type == "Dijkstra") {
                print("Dijkstra", dijkstra(p.graph, p.start, p.goal));
            } else if (a.type == "AStar") {
                // Generic graphs carry no coordinates, so A* runs with the zero heuristic
                // (equivalent to Dijkstra). See examples/grid_astar.cpp for a real heuristic.
                print("AStar", astar<double>(p.graph, p.start, p.goal,
                                             [](NodeId, NodeId) { return 0.0; }));
            } else {
                std::cerr << "Unknown search algorithm: " << a.type << '\n';
                return 2;
            }
        }
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << '\n';
        return 1;
    }
}
