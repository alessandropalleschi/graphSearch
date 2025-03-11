#include "dijkstra.hpp"

// Constructor for Dijkstra class
template <typename CostType>
Dijkstra<CostType>::Dijkstra(const Graph<CostType>& graph) : SearchAlgorithm<CostType>(graph) {}

// Dijkstra's search algorithm implementation
template <typename CostType>
typename Dijkstra<CostType>::Path Dijkstra<CostType>::search(Node<CostType>& start, Node<CostType>& goal) {
    Path path;
    using NodeType = Node<CostType>;
    using NodePtr = std::shared_ptr<NodeType>;

    std::priority_queue<NodeType, std::vector<NodeType>, Compare<CostType>> toVisit;
    std::unordered_map<NodeType, CostType> costMap;

    // Initialize start node's cost and add it to the queue
    start.setNodeCost(CostType(0));
    toVisit.push(start);
    costMap[start] = CostType(0);

    while (!toVisit.empty()) {
        NodeType currentNode = toVisit.top();
        toVisit.pop();

        // Skip if this path is outdated
        if (currentNode.costToCome > costMap[currentNode]) {
            continue;
        }

        // Add visited node to the path
        path.push_back(currentNode);

        // Goal reached
        if (currentNode == goal) {
            return path;
        }

        // Explore neighbors
        for (const auto& edge : this->getNeighbours(currentNode)) {
            NodeType adjacentNode = edge.destination;
            CostType newCost = costMap[currentNode] + edge.weight;

            if (costMap.find(adjacentNode) == costMap.end() || newCost < costMap[adjacentNode]) {
                costMap[adjacentNode] = newCost;
                adjacentNode.setNodeCost(newCost);
                adjacentNode.setParent(currentNode);
                adjacentNode.parent = std::make_shared<NodeType>(currentNode); // Proper use of shared_ptr
                toVisit.push(adjacentNode);
            }
        }
    }

    // No path found
    return {};
}

// Explicit instantiation for supported types
template class Dijkstra<int>;
template class Dijkstra<float>;
template class Dijkstra<double>;
