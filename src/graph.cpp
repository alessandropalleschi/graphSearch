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

// Function to calculate the cost of a path
template <typename CostType>
CostType calculatePathCost(const std::vector<Node>& path, const std::vector<Edge<CostType>>& edges) {
    CostType cost = CostType(0);

    for (size_t i = 0; i < path.size() - 1; ++i) {
        // Find the corresponding edge between consecutive nodes
        auto edgeIt = std::find_if(edges.begin(), edges.end(), [&](const Edge<CostType>& edge) {
            return edge.source == path[i] && edge.destination == path[i + 1];
        });

        // If the edge is found, add its weight to the total cost
        if (edgeIt != edges.end()) {
            cost += edgeIt->weight;
        } else {
            // Handle the case where there is no corresponding edge (optional)
            std::cerr << "Error: No edge found between nodes " << path[i].ID << " and " << path[i + 1].ID << std::endl;
            // You may choose to return an error code or throw an exception
            // Alternatively, you can consider adding a default weight for such cases
        }
    }

    return cost;
}
