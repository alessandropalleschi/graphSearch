#ifndef SEARCH_ALGORITHM_HPP
#define SEARCH_ALGORITHM_HPP

#include <vector>
#include <functional>
#include "graph.hpp"
#include <queue>

/**
 * @brief Base class for search algorithms.
 * 
 * This class defines the common interface for search algorithms and provides utility methods.
 */
template <typename CostType>
class SearchAlgorithm {
protected:
    const Graph<CostType>& graph;

public:
    using Path = std::vector<Node<CostType>>;
    SearchAlgorithm(const Graph<CostType>& g);
    std::vector<Edge<CostType>> getNeighbours(const Node<CostType>& src) const;
    virtual Path search(Node<CostType>& start, Node<CostType>& goal) = 0;
};

/**
 * @brief Comparator class for nodes.
 * 
 * This class provides a custom comparison function for nodes, used in priority queues.
 */
template <typename CostType>
class Compare {
public:
    bool operator()(Node<CostType>& a, Node<CostType>& b);
};

#endif // SEARCH_ALGORITHM_HPP
