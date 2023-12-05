#ifndef GRAPH_HPP
#define GRAPH_HPP

#include <unordered_map>
#include <vector>
#include <string>
#include <limits>
#include <functional>
#include <iostream>
#include <memory>

/**
 * @brief Class representing a node in the graph.
 * 
 * This class holds information about a node, including its ID, name, cost, and cost-to-go.
 */
template <typename CostType>
class Node {
public:
    int ID;
    std::string name;
    CostType costToCome;
    CostType costToGo;
    CostType cost;
    std::shared_ptr<Node<CostType>> parent;
    
    Node(const int& ID, const std::string& name, const CostType costToCome = std::numeric_limits<CostType>::max(), const CostType costToGo = CostType(0), std::shared_ptr<Node<CostType>> parent=nullptr);

    // Copy constructor
    Node(const Node<CostType>& other)
        : ID(other.ID), name(other.name), costToCome(other.costToCome), costToGo(other.costToGo) {
        // Deep copy the parent if it exists
        if (other.parent) {
            parent = std::make_shared<Node<CostType>>(*other.parent);
        }
    }

    // Move constructor
    Node(Node<CostType>&& other) noexcept
        : ID(std::exchange(other.ID, 0)),
          name(std::move(other.name)),
          costToCome(std::exchange(other.costToCome, CostType())),
          costToGo(std::exchange(other.costToGo, CostType())),
          parent(std::move(other.parent)) {}

    // Copy assignment operator
    Node& operator=(const Node<CostType>& other) {
        if (this != &other) {
            ID = other.ID;
            name = other.name;
            costToGo = other.costToGo;
            costToCome = other.costToCome;

            // Deep copy the parent if it exists
            if (other.parent) {
                parent = std::make_shared<Node<CostType>>(*other.parent);
            } else {
                parent.reset(); // No parent in the other node, so reset the current parent
            }
        }
        return *this;
    }

    // Function to set the parent, replacing the existing parent
    void setParent(Node<CostType>& Parent);
    void setNodeCost(const CostType& costToCome, const CostType& costToGo);
    void setCostToCome(const CostType& costToCome);
    CostType getNodeCost() const;
    bool operator==(const Node& other) const;

};

// Custom hash function specialization for Node<CostType>
namespace std {
    template <typename CostType>
    struct hash<Node<CostType>> {
        std::size_t operator()(const Node<CostType>& node) const {
            return std::hash<int>{}(node.ID);
        }
    };
}

/**
 * @brief Class representing an edge in the graph.
 * 
 * This class defines an edge with a source node, destination node, and weight.
 */
template <typename CostType>
class Edge {
public:
    const Node<CostType> source;
    const Node<CostType> destination;
    const CostType weight;

    Edge(const Node<CostType>& src, const Node<CostType>& dest, const CostType w);
};

/**
 * @brief Class representing a graph.
 * 
 * This class contains an adjacency list to represent the graph structure and provides methods to add edges.
 */
template <typename CostType>
class Graph {
public:
    std::unordered_map<Node<CostType>, std::vector<Edge<CostType>>> adjList;

    void addEdge(const Node<CostType>& src, const Node<CostType>& dest, CostType weight);
};


#endif // GRAPH_HPP
