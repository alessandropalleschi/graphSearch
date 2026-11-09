// Dependency-free benchmark: Dijkstra vs A* (octile) on random obstacle grids.
#include <algorithm>
#include <chrono>
#include <cstdio>
#include <vector>

#include "grid.hpp"

using namespace graphsearch;
using Clock = std::chrono::steady_clock;

template <typename F>
static double medianMs(F&& f, int reps) {
    std::vector<double> t;
    for (int i = 0; i < reps; ++i) {
        auto s = Clock::now();
        f();
        t.push_back(std::chrono::duration<double, std::milli>(Clock::now() - s).count());
    }
    std::sort(t.begin(), t.end());
    return t[t.size() / 2];
}

int main() {
    std::printf("%-10s %-9s %-10s %-12s %-10s %-12s %s\n", "grid", "algo", "median_ms", "expanded",
                "cost", "path_len", "speedup");
    for (std::size_t n : {64, 128, 256, 512, 1024}) {
        auto grid = gridutil::makeGrid(n, n, 0.2, 42);
        const NodeId s = 0, t = n * n - 1;
        auto h = gridutil::octile(n);
        const int reps = n >= 512 ? 5 : 15;

        SearchResult<double> rd, ra;
        double td = medianMs([&] { rd = dijkstra(grid.g, s, t); }, reps);
        double ta = medianMs([&] { ra = astar<double>(grid.g, s, t, h); }, reps);
        char label[32];
        std::snprintf(label, sizeof label, "%zux%zu", n, n);
        std::printf("%-10s %-9s %-10.3f %-12zu %-10.2f %-12zu\n", label, "dijkstra", td, rd.expanded,
                    rd.cost, rd.path.size());
        std::printf("%-10s %-9s %-10.3f %-12zu %-10.2f %-12zu %.2fx\n", label, "astar", ta, ra.expanded,
                    ra.cost, ra.path.size(), td / ta);
    }
}
