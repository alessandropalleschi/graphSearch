// Dependency-free benchmark harness.
//   ./bench [--quick] [--csv | --json]
// Scenarios: random obstacle grids (all algorithms) and random sparse graphs.
#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "grid.hpp"

using namespace graphsearch;
using Clock = std::chrono::steady_clock;

struct Row {
    std::string scenario, algo;
    std::size_t nodes, edges, expanded;
    double ms, cost;
    bool found;
};
static std::vector<Row> rows;

template <typename F>
static void measure(const std::string& scenario, const std::string& algo, const Graph<double>& g,
                    int reps, F&& f) {
    std::vector<double> t;
    SearchResult<double> r;
    for (int i = 0; i < reps; ++i) {
        const auto s = Clock::now();
        r = f();
        t.push_back(std::chrono::duration<double, std::milli>(Clock::now() - s).count());
    }
    std::sort(t.begin(), t.end());
    rows.push_back({scenario, algo, g.size(), g.edgeCount(), r.expanded, t[t.size() / 2], r.cost, r.found});
}

int main(int argc, char** argv) {
    bool quick = false, csv = false, json = false;
    for (int i = 1; i < argc; ++i) {
        if (!std::strcmp(argv[i], "--quick")) quick = true;
        else if (!std::strcmp(argv[i], "--csv")) csv = true;
        else if (!std::strcmp(argv[i], "--json")) json = true;
        else { std::fprintf(stderr, "usage: bench [--quick] [--csv|--json]\n"); return 2; }
    }

    const std::vector<std::size_t> gridSizes = quick ? std::vector<std::size_t>{64, 128}
                                                     : std::vector<std::size_t>{64, 128, 256, 512, 1024};
    for (std::size_t n : gridSizes) {
        const auto grid = gridutil::makeGrid(n, n, 0.2, 42);
        const auto rev = grid.g.reversed();
        const NodeId s = 0, t = n * n - 1;
        const auto h = gridOctile<double>(n);
        const int reps = quick ? 3 : (n >= 512 ? 5 : 15);
        const std::string sc = "grid" + std::to_string(n) + "x" + std::to_string(n);
        measure(sc, "dijkstra", grid.g, reps, [&] { return dijkstra(grid.g, s, t); });
        measure(sc, "bidirectional", grid.g, reps, [&] { return bidirectionalDijkstra(grid.g, rev, s, t); });
        measure(sc, "astar", grid.g, reps, [&] { return astar(grid.g, s, t, h); });
        measure(sc, "weighted_astar_2", grid.g, reps, [&] { return weightedAstar(grid.g, s, t, h, 2.0); });
        measure(sc, "greedy", grid.g, reps, [&] { return greedyBestFirst(grid.g, s, t, h); });
        measure(sc, "bfs", grid.g, reps, [&] { return bfs(grid.g, s, t); });
        if (n <= 128) measure(sc, "bellman_ford", grid.g, reps, [&] { return bellmanFord(grid.g, s, t); });
    }

    const std::vector<std::size_t> graphSizes = quick ? std::vector<std::size_t>{10000}
                                                      : std::vector<std::size_t>{10000, 100000, 500000};
    for (std::size_t n : graphSizes) {
        const auto g = gridutil::makeRandomGraph(n, 4, 1);
        const auto rev = g.reversed();
        const int reps = quick ? 3 : 7;
        const std::string sc = "random" + std::to_string(n);
        measure(sc, "dijkstra", g, reps, [&] { return dijkstra(g, 0, n / 2); });
        measure(sc, "bidirectional", g, reps, [&] { return bidirectionalDijkstra(g, rev, 0, n / 2); });
        measure(sc, "bfs", g, reps, [&] { return bfs(g, 0, n / 2); });
        measure(sc, "bellman_ford", g, reps, [&] { return bellmanFord(g, 0, n / 2); });
    }

    if (csv) {
        std::puts("scenario,algo,nodes,edges,median_ms,expanded,cost,found");
        for (const auto& r : rows)
            std::printf("%s,%s,%zu,%zu,%.4f,%zu,%.4f,%d\n", r.scenario.c_str(), r.algo.c_str(), r.nodes,
                        r.edges, r.ms, r.expanded, r.cost, r.found);
    } else if (json) {
        std::puts("[");
        for (std::size_t i = 0; i < rows.size(); ++i) {
            const auto& r = rows[i];
            std::printf("  {\"scenario\":\"%s\",\"algo\":\"%s\",\"nodes\":%zu,\"edges\":%zu,"
                        "\"median_ms\":%.4f,\"expanded\":%zu,\"cost\":%.4f,\"found\":%s}%s\n",
                        r.scenario.c_str(), r.algo.c_str(), r.nodes, r.edges, r.ms, r.expanded, r.cost,
                        r.found ? "true" : "false", i + 1 < rows.size() ? "," : "");
        }
        std::puts("]");
    } else {
        std::printf("%-14s %-17s %10s %12s %12s %10s\n", "scenario", "algo", "median_ms", "expanded", "cost", "found");
        for (const auto& r : rows)
            std::printf("%-14s %-17s %10.3f %12zu %12.2f %10s\n", r.scenario.c_str(), r.algo.c_str(),
                        r.ms, r.expanded, r.cost, r.found ? "yes" : "no");
    }
}
