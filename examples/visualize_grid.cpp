// Renders Dijkstra / bidirectional / A* / weighted A* / greedy / BFS side by side on one grid.
//   visualize_grid [size=64] [obstacle=0.2] [seed=42] [out=grid_comparison.svg]
#include <fstream>
#include <iostream>

#include "../benchmarks/grid.hpp"
#include "graphsearch/svg.hpp"

using namespace graphsearch;

int main(int argc, char** argv) {
    const std::size_t n = argc > 1 ? std::stoul(argv[1]) : 64;
    const double obstacle = argc > 2 ? std::stod(argv[2]) : 0.2;
    const unsigned seed = argc > 3 ? std::stoul(argv[3]) : 42;
    const std::string out = argc > 4 ? argv[4] : "grid_comparison.svg";

    const auto grid = gridutil::makeGrid(n, n, obstacle, seed);
    const NodeId s = 0, t = n * n - 1;
    const auto h = gridOctile<double>(n);

    std::vector<svg::GridPanel> panels;
    auto add = [&](const std::string& title, auto&& run) {
        svg::GridPanel p;
        p.title = title;
        const SearchResult<double> r = run(&p.trace);
        p.found = r.found; p.path = r.path; p.cost = r.cost; p.expanded = r.expanded;
        panels.push_back(std::move(p));
    };
    add("Dijkstra", [&](auto* tr) { return dijkstra(grid.g, s, t, tr); });
    add("Bidirectional Dijkstra", [&](auto* tr) { return bidirectionalDijkstra(grid.g, s, t, tr); });
    add("BFS (fewest edges, ignores weights)", [&](auto* tr) { return bfs(grid.g, s, t, tr); });
    add("A* (octile)", [&](auto* tr) { return astar(grid.g, s, t, h, tr); });
    add("Weighted A* (w = 2)", [&](auto* tr) { return weightedAstar(grid.g, s, t, h, 2.0, tr); });
    add("Greedy best-first", [&](auto* tr) { return greedyBestFirst(grid.g, s, t, h, tr); });

    const double cell = n <= 64 ? 5 : (n <= 128 ? 3 : 2);
    std::ofstream(out) << svg::gridComparison(
        n, n, grid.blocked, panels, 3, cell,
        "Search algorithms on a " + std::to_string(n) + "×" + std::to_string(n) + " grid",
        std::to_string(int(obstacle * 100)) + "% random obstacles, 8-connected, start top-left → goal bottom-right. "
        "Cells are shaded by when they were expanded.");
    std::cout << "wrote " << out << '\n';
}
