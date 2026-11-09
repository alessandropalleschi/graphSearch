#pragma once

#include <algorithm>
#include <functional>
#include <limits>
#include <queue>
#include <stdexcept>
#include <utility>
#include <vector>

#include "graph.hpp"
#include "heuristics.hpp"

namespace graphsearch {

template <CostType Cost>
struct SearchResult {
    bool found = false;
    std::vector<NodeId> path;     ///< start ... goal (empty if not found)
    Cost cost = Cost(0);          ///< total path cost (BFS: number of edges)
    std::size_t expanded = 0;     ///< nodes expanded (Bellman-Ford: relaxation passes)
    bool negativeCycle = false;   ///< Bellman-Ford only: reachable negative cycle detected
};

namespace detail {

constexpr NodeId none = std::numeric_limits<NodeId>::max();

template <CostType Cost>
void validate(const Graph<Cost>& g, NodeId start, NodeId goal) {
    if (!g.finalized()) throw std::logic_error("Graph::finalize() must be called before searching");
    if (start >= g.size() || goal >= g.size()) throw std::out_of_range("start/goal out of range");
}

template <CostType Cost>
void requireNonNegative(const Graph<Cost>& g) {
    if (g.hasNegativeWeights())
        throw std::invalid_argument("graph has negative weights; use bellmanFord");
}

inline std::vector<NodeId> buildPath(const std::vector<NodeId>& parent, NodeId goal) {
    std::vector<NodeId> path;
    for (NodeId v = goal; v != none; v = parent[v]) path.push_back(v);
    std::reverse(path.begin(), path.end());
    return path;
}

template <typename P, typename C>
struct Entry {
    P prio;
    C g;
    NodeId node;
    // Min-heap on prio; among equal priorities prefer the deeper node (larger g).
    bool operator>(const Entry& o) const { return prio != o.prio ? prio > o.prio : g < o.g; }
};
template <typename P, typename C>
using MinQueue = std::priority_queue<Entry<P, C>, std::vector<Entry<P, C>>, std::greater<>>;

// Shared best-first engine: priority = g + weight * h (or h alone when greedy).
// Stale queue entries are skipped by comparing against the best known g, so nodes are
// re-opened if a cheaper route is found later (needed for inconsistent heuristics).
template <CostType Cost>
SearchResult<Cost> bestFirst(const Graph<Cost>& g, NodeId start, NodeId goal,
                             const Heuristic<Cost>* h, double weight, bool greedy) {
    validate(g, start, goal);
    requireNonNegative(g);
    SearchResult<Cost> res;
    const Cost inf = std::numeric_limits<Cost>::max();
    std::vector<Cost> dist(g.size(), inf);
    std::vector<NodeId> parent(g.size(), none);

    auto prio = [&](Cost gv, NodeId v) {
        const double hv = h ? static_cast<double>((*h)(v, goal)) : 0.0;
        return greedy ? hv : static_cast<double>(gv) + weight * hv;
    };

    MinQueue<double, Cost> open;
    dist[start] = Cost(0);
    open.push({prio(Cost(0), start), Cost(0), start});

    while (!open.empty()) {
        const auto cur = open.top();
        open.pop();
        if (cur.g > dist[cur.node]) continue;  // stale
        ++res.expanded;
        if (cur.node == goal) {
            res.found = true;
            res.cost = cur.g;
            res.path = buildPath(parent, goal);
            return res;
        }
        for (const auto& e : g.neighbours(cur.node)) {
            const Cost nd = cur.g + e.weight;
            if (nd < dist[e.to]) {
                dist[e.to] = nd;
                parent[e.to] = cur.node;
                open.push({prio(nd, e.to), nd, e.to});
            }
        }
    }
    return res;
}

}  // namespace detail

/// Dijkstra's shortest path, O((V+E) log V). Non-negative weights only.
template <CostType Cost>
SearchResult<Cost> dijkstra(const Graph<Cost>& g, NodeId start, NodeId goal) {
    return detail::bestFirst<Cost>(g, start, goal, nullptr, 1.0, false);
}

/// A*: optimal when the heuristic is admissible.
template <CostType Cost>
SearchResult<Cost> astar(const Graph<Cost>& g, NodeId start, NodeId goal, const Heuristic<Cost>& h) {
    return detail::bestFirst<Cost>(g, start, goal, &h, 1.0, false);
}

/// Weighted A* (priority g + w*h, w >= 1): expands fewer nodes; cost <= w * optimal.
template <CostType Cost>
SearchResult<Cost> weightedAstar(const Graph<Cost>& g, NodeId start, NodeId goal,
                                 const Heuristic<Cost>& h, double weight) {
    if (weight < 1.0) throw std::invalid_argument("weight must be >= 1");
    return detail::bestFirst<Cost>(g, start, goal, &h, weight, false);
}

/// Greedy best-first search (priority h only): fast, not optimal.
template <CostType Cost>
SearchResult<Cost> greedyBestFirst(const Graph<Cost>& g, NodeId start, NodeId goal,
                                   const Heuristic<Cost>& h) {
    return detail::bestFirst<Cost>(g, start, goal, &h, 1.0, true);
}

/// Breadth-first search: fewest edges, weights ignored. `cost` is the edge count.
template <CostType Cost>
SearchResult<Cost> bfs(const Graph<Cost>& g, NodeId start, NodeId goal) {
    detail::validate(g, start, goal);
    SearchResult<Cost> res;
    std::vector<NodeId> parent(g.size(), detail::none);
    std::vector<char> seen(g.size(), 0);
    std::queue<NodeId> q;
    seen[start] = 1;
    q.push(start);
    while (!q.empty()) {
        const NodeId u = q.front();
        q.pop();
        ++res.expanded;
        if (u == goal) {
            res.found = true;
            res.path = detail::buildPath(parent, goal);
            res.cost = static_cast<Cost>(res.path.size() - 1);
            return res;
        }
        for (const auto& e : g.neighbours(u)) {
            if (seen[e.to]) continue;
            seen[e.to] = 1;
            parent[e.to] = u;
            q.push(e.to);
        }
    }
    return res;
}

/// Bellman-Ford: handles negative weights. If a negative cycle is reachable from `start`,
/// returns found = false and negativeCycle = true. O(V * E).
template <CostType Cost>
SearchResult<Cost> bellmanFord(const Graph<Cost>& g, NodeId start, NodeId goal) {
    detail::validate(g, start, goal);
    SearchResult<Cost> res;
    const Cost inf = std::numeric_limits<Cost>::max();
    std::vector<Cost> dist(g.size(), inf);
    std::vector<NodeId> parent(g.size(), detail::none);
    dist[start] = Cost(0);

    for (std::size_t pass = 1; pass <= g.size(); ++pass) {
        bool changed = false;
        ++res.expanded;
        for (NodeId u = 0; u < g.size(); ++u) {
            if (dist[u] == inf) continue;
            for (const auto& e : g.neighbours(u)) {
                const Cost nd = dist[u] + e.weight;
                if (nd < dist[e.to]) {
                    dist[e.to] = nd;
                    parent[e.to] = u;
                    changed = true;
                }
            }
        }
        if (!changed) break;
        if (pass == g.size()) {
            res.negativeCycle = true;
            return res;
        }
    }
    if (dist[goal] != inf) {
        res.found = true;
        res.cost = dist[goal];
        res.path = detail::buildPath(parent, goal);
    }
    return res;
}

/// Bidirectional Dijkstra given the reversed graph (g.reversed()); reuse it across many queries.
template <CostType Cost>
SearchResult<Cost> bidirectionalDijkstra(const Graph<Cost>& g, const Graph<Cost>& rev,
                                         NodeId start, NodeId goal) {
    detail::validate(g, start, goal);
    detail::requireNonNegative(g);
    if (rev.size() != g.size() || !rev.finalized()) throw std::invalid_argument("bad reverse graph");
    using detail::none;
    SearchResult<Cost> res;
    if (start == goal) {
        res.found = true;
        res.path = {start};
        res.expanded = 1;
        return res;
    }
    const Cost inf = std::numeric_limits<Cost>::max();
    std::vector<Cost> dist[2] = {std::vector<Cost>(g.size(), inf), std::vector<Cost>(g.size(), inf)};
    std::vector<NodeId> parent[2] = {std::vector<NodeId>(g.size(), none),
                                     std::vector<NodeId>(g.size(), none)};
    detail::MinQueue<Cost, Cost> q[2];
    const Graph<Cost>* graph[2] = {&g, &rev};
    dist[0][start] = Cost(0);
    q[0].push({Cost(0), Cost(0), start});
    dist[1][goal] = Cost(0);
    q[1].push({Cost(0), Cost(0), goal});

    Cost mu = inf;
    NodeId meet = none;
    auto dropStale = [&](int s) {
        while (!q[s].empty() && q[s].top().g > dist[s][q[s].top().node]) q[s].pop();
    };
    for (;;) {
        dropStale(0);
        dropStale(1);
        if (q[0].empty() || q[1].empty()) break;
        if (mu != inf && q[0].top().prio + q[1].top().prio >= mu) break;
        const int s = q[0].top().prio <= q[1].top().prio ? 0 : 1;
        const auto cur = q[s].top();
        q[s].pop();
        ++res.expanded;
        for (const auto& e : graph[s]->neighbours(cur.node)) {
            const Cost nd = cur.g + e.weight;
            if (nd < dist[s][e.to]) {
                dist[s][e.to] = nd;
                parent[s][e.to] = cur.node;
                q[s].push({nd, nd, e.to});
                if (dist[1 - s][e.to] != inf && nd + dist[1 - s][e.to] < mu) {
                    mu = nd + dist[1 - s][e.to];
                    meet = e.to;
                }
            }
        }
    }
    if (meet == none) return res;
    res.found = true;
    res.cost = mu;
    res.path = detail::buildPath(parent[0], meet);  // start ... meet
    for (NodeId v = parent[1][meet]; v != none; v = parent[1][v]) res.path.push_back(v);  // ... goal
    return res;
}

template <CostType Cost>
SearchResult<Cost> bidirectionalDijkstra(const Graph<Cost>& g, NodeId start, NodeId goal) {
    detail::validate(g, start, goal);
    return bidirectionalDijkstra(g, g.reversed(), start, goal);
}

}  // namespace graphsearch
