#pragma once
// Shared helpers: grid graphs with obstacles, used by examples, tests and benchmarks.
#include <cmath>
#include <cstdint>
#include <random>

#include "graphsearch/graph.hpp"
#include "graphsearch/heuristics.hpp"
#include "graphsearch/search.hpp"

namespace gridutil {

struct Grid {
    std::size_t w, h;
    graphsearch::Graph<double> g;
    std::vector<char> blocked;
    std::size_t id(std::size_t x, std::size_t y) const { return y * w + x; }
};

/// 8-connected grid; each cell blocked with probability `obstacle`. Corners stay free.
inline Grid makeGrid(std::size_t w, std::size_t h, double obstacle, std::uint32_t seed) {
    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> u(0.0, 1.0);
    std::vector<char> blocked(w * h, 0);
    for (auto& b : blocked) b = u(rng) < obstacle;
    blocked[0] = blocked[w * h - 1] = 0;
    Grid grid{w, h, graphsearch::Graph<double>(w * h), blocked};
    for (std::size_t y = 0; y < h; ++y)
        for (std::size_t x = 0; x < w; ++x) {
            if (blocked[grid.id(x, y)]) continue;
            for (int dy = -1; dy <= 1; ++dy)
                for (int dx = -1; dx <= 1; ++dx) {
                    if (!dx && !dy) continue;
                    long nx = long(x) + dx, ny = long(y) + dy;
                    if (nx < 0 || ny < 0 || nx >= long(w) || ny >= long(h)) continue;
                    if (blocked[grid.id(nx, ny)]) continue;
                    grid.g.addEdge(grid.id(x, y), grid.id(nx, ny), dx && dy ? std::sqrt(2.0) : 1.0);
                }
        }
    grid.g.finalize();
    return grid;
}

/// Random sparse digraph: `degree` random out-edges per node (weights 1..10) plus a slow
/// chain (weight 20) so that every node is reachable from node 0.
inline graphsearch::Graph<double> makeRandomGraph(std::size_t n, int degree, std::uint32_t seed) {
    std::mt19937 rng(seed);
    graphsearch::Graph<double> g(n);
    for (std::size_t u = 0; u < n; ++u) {
        if (u + 1 < n) g.addEdge(u, u + 1, 20);
        for (int k = 0; k < degree; ++k) g.addEdge(u, rng() % n, 1 + rng() % 10);
    }
    g.finalize();
    return g;
}

}  // namespace gridutil
