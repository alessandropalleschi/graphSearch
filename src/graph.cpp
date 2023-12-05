#include "graph.hpp"

// Node Implementation
template <typename CostType>
Node<CostType>::Node(const int& ID, const std::string& name, const CostType costToCome, const CostType costToGo, std::shared_ptr<Node<CostType>> parent)
        : ID(ID), name(name), costToCome(costToCome), costToGo(costToGo), parent(parent) {}

template <typename CostType>
// Function to set the parent, replacing the existing parent
void Node<CostType>::setParent(Node<CostType>& newParent) {
    parent = std::make_shared<Node<CostType>>(newParent);
}

template <typename CostType>
void Node<CostType>::setNodeCost(const CostType& costToCome, const CostType& costToGo=0) {
    this->costToCome = costToCome;
    this->costToGo = costToGo;
    this->cost = this->costToCome+this->costToGo;
}

template <typename CostType>
void Node<CostType>::setCostToCome(const CostType& costToCome) {
    this->costToCome = costToCome;
}

template <typename CostType>
CostType Node<CostType>::getNodeCost() const {
    return cost;
}

template <typename CostType>
bool Node<CostType>::operator==(const Node& other) const {
    return ID == other.ID && name == other.name;
}

template class Node<int>; // Instantiate for the specific type you are using
template class Node<double>; // Instantiate for the specific type you are using
template class Node<float>; // Instantiate for the specific type you are using

// Edge Implementation
template <typename CostType>
Edge<CostType>::Edge(const Node<CostType>& src, const Node<CostType>& dest, const CostType w)
    : source(src), destination(dest), weight(w) {}

template class Edge<int>; // Explicit instantiation for int
template class Edge<double>; // Explicit instantiation for int
template class Edge<float>; // Explicit instantiation for int

// Graph Implementation
template <typename CostType>
void Graph<CostType>::addEdge(const Node<CostType>& src, const Node<CostType>& dest, CostType weight) {
    adjList[src].emplace_back(src, dest, weight);
}

template class Graph<int>; // Explicit instantiation for int
template class Graph<double>; // Explicit instantiation for int
template class Graph<float>; // Explicit instantiation for int

// Function to calculate the cost of a path
template <typename CostType>
CostType calculatePathCost(const std::vector<Node<CostType>>& path, const Graph<CostType>& graph) {
    CostType cost = CostType(0);

    for (size_t i = 0; i < path.size() - 1; ++i) {
        const Node<CostType> currentNode = path[i];
        const Node<CostType> nextNode = path[i + 1];

        // Find the corresponding edge in the adjacency list
        auto edgeIt = std::find_if(
            graph.adjList[currentNode].begin(),
            graph.adjList[currentNode].end(),
            [&](const Edge<CostType>& edge) {
                return edge.destination == nextNode;
            });

        // If the edge is found, add its weight to the total cost
        if (edgeIt != graph.adjList[currentNode].end()) {
            cost += edgeIt->weight;
        } else {
            // Handle the case where there is no corresponding edge (optional)
            std::cerr << "Error: No edge found between nodes " << currentNode.ID << " and " << nextNode.ID << std::endl;
            // You may choose to return an error code or throw an exception
            // Alternatively, you can consider adding a default weight for such cases
        }
    }

    return cost;
}

