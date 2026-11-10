#include <fstream>
#include <iostream>

#include "graphsearch/svg.hpp"

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
    std::string file = "config/graph.yaml", svgOut;
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "-h" || a == "--help") {
            std::cout << "usage: graphSearch [config.yaml] [--svg out.svg]\n"
                         "  --svg  draw the graph (needs a `nodes:` section) and highlight the first path found\n";
            return 0;
        }
        if (a == "--svg" && i + 1 < argc) svgOut = argv[++i];
        else file = a;
    }
    try {
        const Problem p = loadProblem(file);
        std::vector<NodeId> shown;
        std::string shownBy;
        for (const auto& spec : p.algorithms) {
            const auto r = run(p, spec);
            print(spec, r);
            if (shown.empty() && r.found) { shown = r.path; shownBy = spec.type; }
        }
        if (!svgOut.empty()) {
            if (!p.coords) throw std::invalid_argument("--svg needs a `nodes:` section with coordinates");
            std::ofstream(svgOut) << svg::graphDiagram(
                p.graph, *p.coords, shown,
                shown.empty() ? "Graph" : "Shortest path found by " + shownBy);
            std::cout << "wrote " << svgOut << '\n';
        }
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << '\n';
        return 1;
    }
}
