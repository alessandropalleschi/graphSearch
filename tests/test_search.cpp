#include <cmath>
#include <random>

#include "../benchmarks/grid.hpp"
#include "graphsearch/svg.hpp"
#include "check.hpp"

using namespace graphsearch;

static const double INF = 1e18;

// All-pairs reference (Floyd-Warshall) for small graphs.
static std::vector<std::vector<double>> floyd(const Graph<double>& g) {
    const std::size_t n = g.size();
    std::vector<std::vector<double>> d(n, std::vector<double>(n, INF));
    for (NodeId u = 0; u < n; ++u) {
        d[u][u] = 0;
        for (const auto& e : g.neighbours(u)) d[u][e.to] = std::min(d[u][e.to], e.weight);
    }
    for (std::size_t k = 0; k < n; ++k)
        for (std::size_t i = 0; i < n; ++i)
            for (std::size_t j = 0; j < n; ++j)
                if (d[i][k] < INF && d[k][j] < INF) d[i][j] = std::min(d[i][j], d[i][k] + d[k][j]);
    return d;
}

static Graph<double> randomSmall(std::mt19937& rng, std::size_t n, std::size_t m, bool negative) {
    Graph<double> g(n);
    std::vector<int> pot(n);
    for (auto& p : pot) p = int(rng() % 20);
    for (std::size_t i = 0; i < m; ++i) {
        NodeId a = rng() % n, b = rng() % n;
        double w = double(rng() % 10);
        // Reduced-cost trick: w + pot[b] - pot[a] can be negative but no cycle can be negative.
        if (negative) w += pot[b] - pot[a];
        g.addEdge(a, b, w);
    }
    g.finalize();
    return g;
}

static bool validPath(const Graph<double>& g, const SearchResult<double>& r, NodeId s, NodeId t) {
    if (r.path.empty() || r.path.front() != s || r.path.back() != t) return false;
    double c = 0;
    for (std::size_t i = 0; i + 1 < r.path.size(); ++i) {
        double best = INF;
        for (const auto& e : g.neighbours(r.path[i]))
            if (e.to == r.path[i + 1]) best = std::min(best, e.weight);
        if (best == INF) return false;
        c += best;
    }
    return std::abs(c - r.cost) < 1e-9;
}

TEST(sample_graph) {
    Graph<double> g(5);
    g.addEdge(0, 1, 1); g.addEdge(1, 2, 2); g.addEdge(2, 3, 1);
    g.addEdge(3, 4, 3); g.addEdge(0, 3, 2);
    g.finalize();
    auto r = dijkstra(g, 0, 4);
    CHECK(r.found && r.cost == 5);
    CHECK((r.path == std::vector<NodeId>{0, 3, 4}));
    CHECK(astar(g, 0, 4, zeroHeuristic<double>()).cost == 5);
    CHECK(bidirectionalDijkstra(g, 0, 4).cost == 5);
    CHECK(bellmanFord(g, 0, 4).cost == 5);
    CHECK(bfs(g, 0, 4).cost == 2);
}

TEST(edge_cases) {
    Graph<int> g(3);
    g.addEdge(0, 1, 4);
    g.finalize();
    CHECK(dijkstra(g, 0, 0).found && dijkstra(g, 0, 0).cost == 0);  // start == goal
    CHECK(bidirectionalDijkstra(g, 0, 0).found);
    CHECK(!dijkstra(g, 0, 2).found && dijkstra(g, 0, 2).path.empty());  // unreachable
    CHECK(!bidirectionalDijkstra(g, 0, 2).found);
    CHECK(!bfs(g, 0, 2).found);
    CHECK(!bellmanFord(g, 0, 2).found);
    CHECK(dijkstra(g, 0, 1).cost == 4);  // integer costs
}

TEST(input_validation) {
    Graph<double> g(3);
    CHECK_THROWS(g.addEdge(0, 5, 1), std::out_of_range);
    g.addEdge(0, 1, -1);
    CHECK_THROWS(dijkstra(g, 0, 1), std::logic_error);  // not finalized
    g.finalize();
    CHECK_THROWS(dijkstra(g, 0, 1), std::invalid_argument);  // negative weight
    CHECK_THROWS(bellmanFord(g, 9, 0), std::out_of_range);
    CHECK_THROWS(weightedAstar(g, 0, 1, zeroHeuristic<double>(), 0.5), std::invalid_argument);
    CHECK(bellmanFord(g, 0, 1).cost == -1);
}

TEST(csr_incremental_finalize) {
    Graph<double> g(3);
    g.addEdge(0, 1, 1);
    g.addEdge(0, 2, 5);
    g.finalize();
    g.addEdge(1, 2, 1);
    g.addEdge(0, 2, 9);
    CHECK(!g.finalized());
    g.finalize();
    CHECK(g.edgeCount() == 4);
    CHECK(g.neighbours(0).size() == 3);
    CHECK(g.neighbours(0)[0].to == 1 && g.neighbours(0)[2].weight == 9);  // insertion order kept
    CHECK(dijkstra(g, 0, 2).cost == 2);
}

TEST(matches_floyd_warshall) {
    std::mt19937 rng(123);
    for (int iter = 0; iter < 200; ++iter) {
        const std::size_t n = 2 + rng() % 12;
        auto g = randomSmall(rng, n, rng() % (n * 3), false);
        auto ref = floyd(g);
        const auto rev = g.reversed();
        for (NodeId s = 0; s < n; ++s)
            for (NodeId t = 0; t < n; ++t) {
                const bool reach = ref[s][t] < INF;
                for (const auto& r : {dijkstra(g, s, t), bidirectionalDijkstra(g, rev, s, t),
                                      bellmanFord(g, s, t), astar(g, s, t, zeroHeuristic<double>())}) {
                    CHECK(r.found == reach);
                    if (reach) {
                        CHECK(std::abs(r.cost - ref[s][t]) < 1e-9);
                        CHECK(validPath(g, r, s, t));
                    }
                }
                CHECK(bfs(g, s, t).found == reach);
            }
    }
}

TEST(bellman_ford_negative_weights) {
    std::mt19937 rng(7);
    int sawNegative = 0;
    for (int iter = 0; iter < 200; ++iter) {
        const std::size_t n = 2 + rng() % 10;
        auto g = randomSmall(rng, n, rng() % (n * 3), true);
        sawNegative += g.hasNegativeWeights();
        auto ref = floyd(g);
        for (NodeId s = 0; s < n; ++s)
            for (NodeId t = 0; t < n; ++t) {
                auto r = bellmanFord(g, s, t);
                CHECK(!r.negativeCycle);
                CHECK(r.found == (ref[s][t] < INF));
                if (r.found) CHECK(std::abs(r.cost - ref[s][t]) < 1e-9);
            }
    }
    CHECK(sawNegative > 20);
}

TEST(bellman_ford_negative_cycle) {
    Graph<double> g(3);
    g.addEdge(0, 1, 1); g.addEdge(1, 2, -3); g.addEdge(2, 1, 1);
    g.finalize();
    auto r = bellmanFord(g, 0, 2);
    CHECK(r.negativeCycle && !r.found);
    Graph<double> loop(1);
    loop.addEdge(0, 0, -1);
    loop.finalize();
    CHECK(bellmanFord(loop, 0, 0).negativeCycle);
}

TEST(astar_matches_dijkstra_on_grids) {
    for (unsigned seed = 0; seed < 50; ++seed) {
        auto grid = gridutil::makeGrid(30, 30, 0.3, seed);
        const NodeId t = 30 * 30 - 1;
        auto d = dijkstra(grid.g, 0, t);
        auto a = astar(grid.g, 0, t, gridOctile<double>(30));
        auto b = bidirectionalDijkstra(grid.g, 0, t);
        CHECK(d.found == a.found && d.found == b.found);
        if (d.found) {
            CHECK(std::abs(d.cost - a.cost) < 1e-9 && std::abs(d.cost - b.cost) < 1e-9);
            CHECK(a.expanded <= d.expanded);
            CHECK(validPath(grid.g, a, 0, t) && validPath(grid.g, b, 0, t));
        }
    }
}

// Admissible but inconsistent heuristic: a closed-set A* would return 4.5; re-opening gives 4.
TEST(astar_inconsistent_heuristic_is_optimal) {
    Graph<double> g(5);  // 0=S 1=A 2=C 3=B 4=G
    g.addEdge(0, 1, 1); g.addEdge(1, 2, 1); g.addEdge(2, 3, 1);
    g.addEdge(0, 3, 3.5); g.addEdge(3, 4, 1);
    g.finalize();
    const double hv[] = {4, 3, 2, 0, 0};
    Heuristic<double> h = [&](NodeId n, NodeId) { return hv[n]; };
    auto r = astar(g, 0, 4, h);
    CHECK(r.found && r.cost == 4);
    CHECK((r.path == std::vector<NodeId>{0, 1, 2, 3, 4}));
}

TEST(weighted_and_greedy_bounds) {
    for (unsigned seed = 0; seed < 30; ++seed) {
        auto grid = gridutil::makeGrid(30, 30, 0.25, seed);
        const NodeId t = 30 * 30 - 1;
        auto h = gridOctile<double>(30);
        auto opt = dijkstra(grid.g, 0, t);
        auto w = weightedAstar(grid.g, 0, t, h, 2.0);
        auto gr = greedyBestFirst(grid.g, 0, t, h);
        CHECK(opt.found == w.found && opt.found == gr.found);
        if (!opt.found) continue;
        CHECK(w.cost >= opt.cost - 1e-9 && w.cost <= 2.0 * opt.cost + 1e-9);
        CHECK(gr.cost >= opt.cost - 1e-9);
        CHECK(validPath(grid.g, w, 0, t) && validPath(grid.g, gr, 0, t));
    }
}

TEST(heuristics_from_coordinates) {
    auto pts = std::make_shared<const std::vector<Point>>(std::vector<Point>{{0, 0}, {3, 4}, {1, 1}});
    CHECK(euclidean<double>(pts)(0, 1) == 5.0);
    CHECK(manhattan<double>(pts)(0, 1) == 7.0);
    CHECK(octile<double>(pts)(0, 2) == std::sqrt(2.0));
    CHECK(euclidean<int>(pts)(0, 1) == 5);
}

TEST(trace_records_expansions_in_order) {
    auto grid = gridutil::makeGrid(20, 20, 0.2, 3);
    std::vector<NodeId> tr;
    auto r = dijkstra(grid.g, 0, 399, &tr);
    CHECK(r.found && tr.size() == r.expanded && tr.front() == 0 && tr.back() == 399);
    std::vector<NodeId> tb;
    auto b = bidirectionalDijkstra(grid.g, 0, 399, &tb);
    CHECK(tb.size() == b.expanded);
    std::vector<NodeId> tf;
    bfs(grid.g, 0, 399, &tf);
    CHECK(!tf.empty() && tf.front() == 0);
}

TEST(svg_output_is_well_formed) {
    auto grid = gridutil::makeGrid(16, 16, 0.2, 1);
    svg::GridPanel p;
    p.title = "A & B <x>";
    auto r = dijkstra(grid.g, 0, 255, &p.trace);
    p.found = r.found; p.path = r.path; p.cost = r.cost; p.expanded = r.expanded;
    const std::string s = svg::gridComparison(16, 16, grid.blocked, {p}, 1, 5, "t", "sub");
    CHECK(s.rfind("<svg", 0) == 0 && s.find("</svg>") != std::string::npos);
    CHECK(s.find("<polyline") != std::string::npos);
    CHECK(s.find("A &amp; B &lt;x&gt;") != std::string::npos);  // escaped
    std::vector<Point> pts{{0, 0}, {1, 1}};
    Graph<double> g(2);
    g.addEdge(0, 1, 2);
    g.finalize();
    CHECK(svg::graphDiagram(g, pts, {0, 1}, "g").find("<line") != std::string::npos);
    CHECK(svg::barChart({{"s", {{"a", 1.5, "n", 0}, {"b", 20, "n", 1}}}}, "h", "u").find("<rect") != std::string::npos);
}

int main() { return runAll(); }
