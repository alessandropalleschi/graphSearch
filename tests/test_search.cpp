#include <cstdio>
#include <cstdlib>
#include <random>

#include "../benchmarks/grid.hpp"

using namespace graphsearch;

#define CHECK(c)                                                          \
    do {                                                                  \
        if (!(c)) {                                                       \
            std::fprintf(stderr, "%s:%d: CHECK failed: %s\n", __FILE__, __LINE__, #c); \
            std::exit(1);                                                 \
        }                                                                 \
    } while (0)

static Heuristic<double> zero = [](NodeId, NodeId) { return 0.0; };

static void testSample() {
    Graph<double> g(5);
    g.addEdge(0, 1, 1); g.addEdge(1, 2, 2); g.addEdge(2, 3, 1);
    g.addEdge(3, 4, 3); g.addEdge(0, 3, 2);
    auto r = dijkstra(g, 0, 4);
    CHECK(r.found && r.cost == 5);
    CHECK((r.path == std::vector<NodeId>{0, 3, 4}));
    CHECK(astar<double>(g, 0, 4, zero).cost == 5);
}

static void testEdgeCases() {
    Graph<int> g(3);
    g.addEdge(0, 1, 4);
    CHECK(dijkstra(g, 0, 0).found && dijkstra(g, 0, 0).cost == 0);  // start == goal
    CHECK(!dijkstra(g, 0, 2).found);                                // unreachable
    CHECK(dijkstra(g, 0, 2).path.empty());
    bool threw = false;
    try { g.addEdge(0, 5, 1); } catch (const std::out_of_range&) { threw = true; }
    CHECK(threw);
    threw = false;
    try { g.addEdge(0, 1, -1); } catch (const std::invalid_argument&) { threw = true; }
    CHECK(threw);
    threw = false;
    try { dijkstra(g, 9, 0); } catch (const std::out_of_range&) { threw = true; }
    CHECK(threw);
}

// A* with a consistent heuristic must return the same optimal cost as Dijkstra.
static void testAStarMatchesDijkstra() {
    for (unsigned seed = 0; seed < 50; ++seed) {
        auto grid = gridutil::makeGrid(30, 30, 0.3, seed);
        auto d = dijkstra(grid.g, 0, 30 * 30 - 1);
        auto a = astar<double>(grid.g, 0, 30 * 30 - 1, gridutil::octile(30));
        CHECK(d.found == a.found);
        if (d.found) {
            CHECK(std::abs(d.cost - a.cost) < 1e-9);
            CHECK(a.expanded <= d.expanded);
        }
    }
}

int main() {
    testSample();
    testEdgeCases();
    testAStarMatchesDijkstra();
    std::puts("all tests passed");
}
