#pragma once
// Shared helpers: grid graphs with obstacles, used by examples and benchmarks.
#include <cmath>
#include <cstdint>
#include <random>

#include "graphsearch/graph.hpp"
#include "graphsearch/search.hpp"

namespace gridutil {

struct Grid {
    std::size_t w, h;
    graphsearch::Graph<double> g;
    std::size_t id(std::size_t x, std::size_t y) const { return y * w + x; }
};

/// 8-connected grid; each cell blocked with probability `obstacle`. Start/goal corners stay free.
inline Grid makeGrid(std::size_t w, std::size_t h, double obstacle, std::uint32_t seed) {
    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> u(0.0, 1.0);
    std::vector<char> blocked(w * h, 0);
    for (auto& b : blocked) b = u(rng) < obstacle;
    blocked[0] = blocked[w * h - 1] = 0;
    Grid grid{w, h, graphsearch::Graph<double>(w * h)};
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
    return grid;
}

/// Octile distance: admissible and consistent for 8-connected grids.
inline graphsearch::Heuristic<double> octile(std::size_t w) {
    return [w](graphsearch::NodeId a, graphsearch::NodeId b) {
        double dx = std::abs(double(a % w) - double(b % w));
        double dy = std::abs(double(a / w) - double(b / w));
        return (dx + dy) + (std::sqrt(2.0) - 2.0) * std::min(dx, dy);
    };
}

}  // namespace gridutil
