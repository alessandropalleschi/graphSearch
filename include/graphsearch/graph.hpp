#pragma once

#include <cstddef>
#include <stdexcept>
#include <vector>

namespace graphsearch {

using NodeId = std::size_t;

/// Directed, weighted edge.
template <typename Cost>
struct Edge {
    NodeId to;
    Cost weight;
};

/// Directed weighted graph stored as an adjacency list over dense node ids [0, size()).
template <typename Cost>
class Graph {
public:
    explicit Graph(std::size_t nodeCount = 0) : adj_(nodeCount) {}

    std::size_t size() const noexcept { return adj_.size(); }

    std::size_t edgeCount() const noexcept {
        std::size_t n = 0;
        for (const auto& e : adj_) n += e.size();
        return n;
    }

    /// Adds a directed edge. Throws on out-of-range ids or negative weights
    /// (Dijkstra / A* require non-negative weights).
    void addEdge(NodeId from, NodeId to, Cost weight) {
        check(from);
        check(to);
        if (weight < Cost(0)) throw std::invalid_argument("negative edge weight");
        adj_[from].push_back({to, weight});
    }

    void addUndirectedEdge(NodeId a, NodeId b, Cost weight) {
        addEdge(a, b, weight);
        addEdge(b, a, weight);
    }

    const std::vector<Edge<Cost>>& neighbours(NodeId n) const {
        check(n);
        return adj_[n];
    }

private:
    void check(NodeId n) const {
        if (n >= adj_.size()) throw std::out_of_range("node id out of range");
    }
    std::vector<std::vector<Edge<Cost>>> adj_;
};

}  // namespace graphsearch
