#ifndef GRAPH_HPP
#define GRAPH_HPP

#include <unordered_map>
#include <vector>
#include <string>
#include <limits>
#include <functional>

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
    CostType cost;
    CostType costToGo;

    Node(const int& ID, const std::string& name, const CostType cost = std::numeric_limits<CostType>::max(), const CostType costToGo = CostType(0));

    void setNodeCost(const CostType& cost, const CostType& costToGo);
    void setNodeCost(const CostType& cost);
    CostType getNodeCost() const;
    bool operator==(const Node& other) const;
};

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

    Edge(const Node<CostType> src, const Node<CostType> dest, const CostType w);
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

    void addEdge(const Node<CostType> src, const Node<CostType> dest, CostType weight);
};

#endif // GRAPH_HPP
