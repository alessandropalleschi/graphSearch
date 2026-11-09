#pragma once

#include <algorithm>
#include <functional>
#include <limits>
#include <queue>
#include <utility>
#include <vector>

#include "graph.hpp"

namespace graphsearch {

template <CostType Cost>
struct SearchResult {
    bool found = false;
    std::vector<NodeId> path;  ///< start ... goal (empty if not found)
    Cost cost = Cost(0);       ///< total path cost
    std::size_t expanded = 0;  ///< number of nodes popped and expanded
};

/// h(node, goal) -> admissible estimate of the remaining cost.
template <CostType Cost>
using Heuristic = std::function<Cost(NodeId, NodeId)>;

namespace detail {

template <CostType Cost>
SearchResult<Cost> run(const Graph<Cost>& g, NodeId start, NodeId goal,
                       const Heuristic<Cost>* h) {
    SearchResult<Cost> res;
    const std::size_t n = g.size();
    if (start >= n || goal >= n) throw std::out_of_range("start/goal out of range");

    constexpr NodeId none = std::numeric_limits<NodeId>::max();
    const Cost inf = std::numeric_limits<Cost>::max();
    std::vector<Cost> dist(n, inf);
    std::vector<NodeId> parent(n, none);
    std::vector<char> closed(n, 0);

    using Entry = std::pair<Cost, NodeId>;  // (priority, node)
    std::priority_queue<Entry, std::vector<Entry>, std::greater<>> open;

    dist[start] = Cost(0);
    open.emplace(h ? (*h)(start, goal) : Cost(0), start);

    while (!open.empty()) {
        const NodeId u = open.top().second;
        open.pop();
        if (closed[u]) continue;  // stale queue entry
        closed[u] = 1;
        ++res.expanded;

        if (u == goal) {
            res.found = true;
            res.cost = dist[u];
            for (NodeId v = goal; v != none; v = parent[v]) res.path.push_back(v);
            std::reverse(res.path.begin(), res.path.end());
            return res;
        }

        for (const auto& e : g.neighbours(u)) {
            if (closed[e.to]) continue;
            const Cost nd = dist[u] + e.weight;
            if (nd < dist[e.to]) {
                dist[e.to] = nd;
                parent[e.to] = u;
                open.emplace(nd + (h ? (*h)(e.to, goal) : Cost(0)), e.to);
            }
        }
    }
    return res;
}

}  // namespace detail

/// Dijkstra's shortest path, O((V+E) log V).
template <CostType Cost>
SearchResult<Cost> dijkstra(const Graph<Cost>& g, NodeId start, NodeId goal) {
    return detail::run<Cost>(g, start, goal, nullptr);
}

/// A* shortest path. The heuristic must be admissible and consistent for optimal results.
template <CostType Cost>
SearchResult<Cost> astar(const Graph<Cost>& g, NodeId start, NodeId goal,
                         const Heuristic<Cost>& h) {
    return detail::run<Cost>(g, start, goal, &h);
}

}  // namespace graphsearch
