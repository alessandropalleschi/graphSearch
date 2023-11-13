#include "searchAlgorithm.hpp"

template <typename CostType>
SearchAlgorithm<CostType>::SearchAlgorithm(const Graph<CostType>& g) : graph(g) {}

template <typename CostType>
std::vector<Edge<CostType>> SearchAlgorithm<CostType>::getNeighbours(const Node<CostType>& src) const {
    auto it = graph.adjList.find(src);
    if (it != graph.adjList.end()) {
        return it->second;
    }
    return {};
}

template class SearchAlgorithm<int>; // Explicit instantiation for int

template <typename CostType>
bool Compare<CostType>::operator()(Node<CostType>& a, Node<CostType>& b) {
    if (a.getNodeCost() == b.getNodeCost()) {
        return a.ID < b.ID;
    }
    return a.getNodeCost() > b.getNodeCost();
}

template class Compare<int>; // Explicit instantiation for int


