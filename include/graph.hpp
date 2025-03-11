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
    CostType costToCome = std::numeric_limits<CostType>::max();
    CostType costToGo = CostType(0);
    CostType cost = CostType(0);
    std::shared_ptr<Node<CostType>> parent = nullptr;

    Node(int ID, const std::string& name, CostType costToCome = std::numeric_limits<CostType>::max(), CostType costToGo = CostType(0), std::shared_ptr<Node<CostType>> parent = nullptr)
        : ID(ID), name(name), costToCome(costToCome), costToGo(costToGo), parent(parent) {}

    // Copy constructor with improved deep copy logic
    Node(const Node<CostType>& other)
        : ID(other.ID), name(other.name), costToCome(other.costToCome), costToGo(other.costToGo), cost(other.cost) {
        if (other.parent) {
            parent = std::make_shared<Node<CostType>>(*other.parent);
        }
    }

    // Move constructor
    Node(Node<CostType>&& other) noexcept = default;

    // Copy assignment operator with improved deep copy logic
    Node& operator=(const Node<CostType>& other) {
        if (this != &other) {
            ID = other.ID;
            name = other.name;
            costToGo = other.costToGo;
            costToCome = other.costToCome;
            cost = other.cost;
            parent = other.parent ? std::make_shared<Node<CostType>>(*other.parent) : nullptr;
        }
        return *this;
    }

    // Set parent node
    void setParent(Node<CostType>& Parent) { parent = std::make_shared<Node<CostType>>(Parent); }

    // Set node cost values
    void setNodeCost(const CostType& costToCome, const CostType& costToGo) {
        this->costToCome = costToCome;
        this->costToGo = costToGo;
        this->cost = costToCome + costToGo;
    }

    void setCostToCome(const CostType& costToCome) { this->costToCome = costToCome; }

    CostType getNodeCost() const { return cost; }

    bool operator==(const Node& other) const { return ID == other.ID; }
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
 */
template <typename CostType>
class Edge {
public:
    Node<CostType> source;
    Node<CostType> destination;
    CostType weight;

    Edge(const Node<CostType>& src, const Node<CostType>& dest, const CostType w)
        : source(src), destination(dest), weight(w) {}
};

/**
 * @brief Class representing a graph.
 */
template <typename CostType>
class Graph {
public:
    std::unordered_map<Node<CostType>, std::vector<Edge<CostType>>> adjList;

    void addEdge(const Node<CostType>& src, const Node<CostType>& dest, CostType weight) {
        adjList[src].emplace_back(src, dest, weight);
    }
};

#endif // GRAPH_HPP
