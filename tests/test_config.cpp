#include <filesystem>
#include <fstream>

#include "../src/config.hpp"
#include "check.hpp"

using namespace graphsearch;

static std::string writeYaml(const char* name, const std::string& body) {
    auto p = std::filesystem::temp_directory_path() / name;
    std::ofstream(p) << body;
    return p.string();
}

static const char* kProblem = R"(
totalNodes: 3
startNode: 0
goalNode: 2
nodes:
  - {id: 0, x: 0, y: 0}
  - {id: 1, x: 1, y: 0}
  - {id: 2, x: 2, y: 0}
edges:
  - {source: 0, destination: 1, weight: 1}
  - {source: 1, destination: 2, weight: 1}
searchAlgorithms:
  - {type: Dijkstra}
  - {type: AStar, heuristic: euclidean}
  - {type: WeightedAStar, heuristic: manhattan, weight: 2}
  - {type: Greedy}
  - {type: BFS}
  - {type: BellmanFord}
  - {type: Bidirectional}
)";

TEST(loads_and_runs_every_algorithm) {
    const Problem p = loadProblem(writeYaml("gs_ok.yaml", kProblem));
    CHECK(p.graph.size() == 3 && p.graph.finalized() && p.coords);
    CHECK(p.algorithms.size() == 7);
    for (const auto& a : p.algorithms) {
        auto r = run(p, a);
        CHECK(r.found && r.cost == 2);
    }
}

TEST(rejects_bad_input) {
    CHECK_THROWS(loadProblem("/nonexistent/file.yaml"), std::exception);
    Problem p = loadProblem(writeYaml("gs_ok2.yaml", kProblem));
    CHECK_THROWS(run(p, {"Nope", "", 1.0}), std::invalid_argument);
    CHECK_THROWS(run(p, {"AStar", "bogus", 1.0}), std::invalid_argument);
    p.coords = nullptr;  // heuristic that needs coordinates, but none given
    CHECK_THROWS(run(p, {"AStar", "euclidean", 1.0}), std::invalid_argument);
    CHECK(run(p, {"AStar", "", 1.0}).found);  // defaults to zero heuristic
}

int main() { return runAll(); }
