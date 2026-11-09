#include <iostream>

#include "../benchmarks/grid.hpp"

int main() {
    auto grid = gridutil::makeGrid(20, 20, 0.2, 7);
    auto r = graphsearch::astar<double>(grid.g, 0, 20 * 20 - 1, graphsearch::gridOctile<double>(20));
    if (!r.found) return std::cout << "no path\n", 0;
    std::cout << "cost " << r.cost << ", " << r.path.size() << " nodes, expanded " << r.expanded << '\n';
}
