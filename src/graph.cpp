#include "graph.hpp"

// Node Implementation
template <typename CostType>
Node<CostType>::Node(const int& ID, const std::string& name, const CostType cost, const CostType costToGo)
    : ID(ID), name(name), cost(cost), costToGo(costToGo) {}

template <typename CostType>
void Node<CostType>::setNodeCost(const CostType& cost, const CostType& costToGo) {
    this->cost = cost;
    this->costToGo = costToGo;
}

template <typename CostType>
void Node<CostType>::setNodeCost(const CostType& cost) {
    this->cost = cost;
}

template <typename CostType>
CostType Node<CostType>::getNodeCost() const {
    return cost + costToGo;
}

template <typename CostType>
bool Node<CostType>::operator==(const Node& other) const {
    return ID == other.ID && name == other.name;
}

template class Node<int>; // Explicit instantiation for int

// Edge Implementation
template <typename CostType>
Edge<CostType>::Edge(const Node<CostType> src, const Node<CostType> dest, const CostType w)
    : source(src), destination(dest), weight(w) {}

template class Edge<int>; // Explicit instantiation for int

// Graph Implementation
template <typename CostType>
void Graph<CostType>::addEdge(const Node<CostType> src, const Node<CostType> dest, CostType weight) {
    adjList[src].emplace_back(src, dest, weight);
}

template class Graph<int>; // Explicit instantiation for int
