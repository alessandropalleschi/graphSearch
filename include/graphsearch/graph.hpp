#pragma once

#include <concepts>
#include <cstddef>
#include <span>
#include <stdexcept>
#include <type_traits>
#include <vector>

namespace graphsearch {

using NodeId = std::size_t;

/// Edge costs must be arithmetic (int, float, double, ...).
template <typename T>
concept CostType = std::is_arithmetic_v<T>;

/// Directed, weighted edge.
template <CostType Cost>
struct Edge {
    NodeId to;
    Cost weight;
};

/// Directed weighted graph over dense node ids [0, size()).
///
/// Edges are collected by addEdge() and packed into CSR (compressed sparse row) form by
/// finalize(), which is cache-friendly for searching. Search functions require a finalized graph.
/// Building is not thread-safe; a finalized graph can be searched concurrently.
template <CostType Cost>
class Graph {
public:
    explicit Graph(std::size_t nodeCount = 0) : n_(nodeCount), offsets_(nodeCount + 1, 0) {}

    std::size_t size() const noexcept { return n_; }
    std::size_t edgeCount() const noexcept { return edges_.size() + pending_.size(); }
    bool finalized() const noexcept { return pending_.empty(); }
    bool hasNegativeWeights() const noexcept { return negative_; }

    /// Adds a directed edge. Throws std::out_of_range on bad ids. Negative weights are allowed
    /// by the graph but rejected by Dijkstra / A* (use bellmanFord).
    void addEdge(NodeId from, NodeId to, Cost weight) {
        check(from);
        check(to);
        if (weight < Cost(0)) negative_ = true;
        pending_.push_back({from, to, weight});
    }

    void addUndirectedEdge(NodeId a, NodeId b, Cost weight) {
        addEdge(a, b, weight);
        addEdge(b, a, weight);
    }

    /// Packs all edges into CSR form. Safe to call repeatedly; edges added later are merged.
    void finalize() {
        if (pending_.empty()) return;
        std::vector<std::size_t> off(n_ + 1, 0);
        for (NodeId u = 0; u < n_; ++u) off[u + 1] = offsets_[u + 1] - offsets_[u];
        for (const auto& e : pending_) ++off[e.from + 1];
        for (NodeId u = 0; u < n_; ++u) off[u + 1] += off[u];

        std::vector<Edge<Cost>> out(edges_.size() + pending_.size());
        std::vector<std::size_t> cur(off.begin(), off.end() - 1);
        for (NodeId u = 0; u < n_; ++u)
            for (std::size_t i = offsets_[u]; i < offsets_[u + 1]; ++i) out[cur[u]++] = edges_[i];
        for (const auto& e : pending_) out[cur[e.from]++] = {e.to, e.weight};

        edges_ = std::move(out);
        offsets_ = std::move(off);
        pending_.clear();
        pending_.shrink_to_fit();
    }

    /// Outgoing edges of `n`. Requires finalize() after the last addEdge().
    std::span<const Edge<Cost>> neighbours(NodeId n) const {
        check(n);
        if (!finalized()) throw std::logic_error("Graph::finalize() must be called first");
        return {edges_.data() + offsets_[n], offsets_[n + 1] - offsets_[n]};
    }

    /// Graph with every edge reversed (finalized). O(V + E).
    Graph reversed() const {
        if (!finalized()) throw std::logic_error("Graph::finalize() must be called first");
        Graph r(n_);
        for (NodeId u = 0; u < n_; ++u)
            for (const auto& e : neighbours(u)) r.addEdge(e.to, u, e.weight);
        r.finalize();
        return r;
    }

private:
    struct Raw {
        NodeId from, to;
        Cost weight;
    };
    void check(NodeId n) const {
        if (n >= n_) throw std::out_of_range("node id out of range");
    }

    std::size_t n_;
    std::vector<std::size_t> offsets_;  // n_ + 1 entries
    std::vector<Edge<Cost>> edges_;     // CSR payload
    std::vector<Raw> pending_;          // edges not yet packed
    bool negative_ = false;
};

}  // namespace graphsearch
