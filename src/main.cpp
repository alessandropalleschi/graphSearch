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
    bool animate = false;
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "-h" || a == "--help") {
            std::cout << "usage: graphSearch [config.yaml] [--svg out.svg [--animate]]\n"
                         "  --svg      draw the graph (needs a `nodes:` section) and highlight the first path found\n"
                         "  --animate  make the SVG step through the first algorithm's expansions\n";
            return 0;
        }
        if (a == "--svg" && i + 1 < argc) svgOut = argv[++i];
        else if (a == "--animate") animate = true;
        else file = a;
    }
    try {
        const Problem p = loadProblem(file);
        std::vector<NodeId> shown, shownTrace;
        std::string shownBy;
        for (const auto& spec : p.algorithms) {
            std::vector<NodeId> trace;
            const auto r = run(p, spec, &trace);
            print(spec, r);
            if (shown.empty() && r.found) { shown = r.path; shownTrace = trace; shownBy = spec.type; }
        }
        if (!svgOut.empty()) {
            if (!p.coords) throw std::invalid_argument("--svg needs a `nodes:` section with coordinates");
            std::ofstream(svgOut) << svg::graphDiagram(
                p.graph, *p.coords, shown,
                shown.empty() ? "Graph" : "Shortest path found by " + shownBy,
                animate ? &shownTrace : nullptr);
            std::cout << "wrote " << svgOut << '\n';
        }
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << '\n';
        return 1;
    }
}
