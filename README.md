# graphSearch

A small, header-only C++20 library implementing **Dijkstra** and **A\*** on weighted directed graphs, with a YAML-driven CLI, tests and benchmarks.

## Layout

```
include/graphsearch/   header-only library (graph.hpp, search.hpp, config.hpp)
src/                   CLI (main.cpp) and YAML loader (config.cpp)
config/graph.yaml      sample problem
tests/                 unit tests (run with ctest)
benchmarks/            Dijkstra vs A* benchmark + grid helpers
examples/              grid_astar.cpp: A* with a custom heuristic
```

## Build

Requires CMake >= 3.14, a C++20 compiler, and yaml-cpp (only for the CLI; disable with `-DGRAPHSEARCH_BUILD_APP=OFF`).

```bash
cmake -S . -B build
cmake --build build -j
ctest --test-dir build --output-on-failure
```

## Use as a library

```cpp
#include "graphsearch/search.hpp"
using namespace graphsearch;

Graph<double> g(5);
g.addEdge(0, 1, 1.0);
// ...
auto r = dijkstra(g, 0, 4);                    // r.found, r.path, r.cost, r.expanded
auto a = astar<double>(g, 0, 4, myHeuristic);  // heuristic: (node, goal) -> estimate
```

Edge weights must be non-negative (enforced). A* is optimal when the heuristic is admissible and consistent.

## CLI

```bash
./build/graphSearch config/graph.yaml
```

```yaml
totalNodes: 5
startNode: 0
goalNode: 4
edges:
  - {source: 0, destination: 1, weight: 1}
searchAlgorithms:
  - type: "Dijkstra"
  - type: "AStar"   # zero heuristic here (no coordinates in the YAML), i.e. same as Dijkstra
```

## Benchmarks

```bash
./build/bench
```

Random N×N 8-connected grids, 20% obstacles, corner to corner, Release build (median of several runs):

| grid | Dijkstra ms | A* ms | Dijkstra expanded | A* expanded | speedup |
|---|---|---|---|---|---|
| 64×64 | 0.28 | 0.015 | 3,308 | 267 | 18.7× |
| 256×256 | 7.8 | 0.98 | 52,586 | 4,094 | 8.0× |
| 1024×1024 | 181 | 31.3 | 839,070 | 77,210 | 5.8× |

Both return identical optimal costs. Numbers vary by machine.

## What changed from v1

The previous version did not compile (mismatched `setNodeCost` signatures, template classes split across `.cpp` files, undeclared `Compare` members) and was incorrect: A* skipped stale entries using the wrong field, and each node deep-copied its whole parent chain. It is now a header-only library using integer node ids, parent arrays, lazy-deletion priority queues, input validation, tests and CI.

## License

MIT, see LICENSE.
