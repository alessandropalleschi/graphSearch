#include "astar.hpp"

// Constructor for AStar class
template <typename CostType>
AStar<CostType>::AStar(const Graph<CostType>& graph, std::function<CostType(const Node<CostType>&, const Node<CostType>&)> heuristic)
    : SearchAlgorithm<CostType>(graph), heuristicFunction(std::move(heuristic)) {}

// A* search algorithm implementation
template <typename CostType>
typename AStar<CostType>::Path AStar<CostType>::search(Node<CostType>& start, Node<CostType>& goal) {
    Path path;
    using NodeType = Node<CostType>;
    using NodePtr = std::shared_ptr<NodeType>;

    std::priority_queue<NodeType, std::vector<NodeType>, Compare<CostType>> toVisit;
    std::unordered_map<NodeType, CostType> costMap;

    start.setNodeCost(CostType(0));
    toVisit.push(start);
    costMap[start] = CostType(0);

    while (!toVisit.empty()) {
        NodeType currentNode = toVisit.top();
        toVisit.pop();

        if (currentNode.costToGo > costMap[currentNode]) {
            continue;
        }

        path.push_back(currentNode);

        if (currentNode == goal) {
            return path;
        }

        for (const auto& edge : this->getNeighbours(currentNode)) {
            NodeType adjacentNode = edge.destination;
            CostType newCost = costMap[currentNode] + edge.weight;

            if (costMap.find(adjacentNode) == costMap.end() || newCost < costMap[adjacentNode]) {
                costMap[adjacentNode] = newCost;
                adjacentNode.setNodeCost(newCost, getHeuristic(adjacentNode, goal));
                adjacentNode.setParent(currentNode);
                adjacentNode.parent = std::make_shared<NodeType>(currentNode);
                toVisit.push(adjacentNode);
            }
        }
    }

    return {};
}

// Explicit instantiation for supported types
template class AStar<int>;
template class AStar<float>;
template class AStar<double>;

template <typename CostType>
CostType AStar<CostType>::getHeuristic(const Node<CostType>& node, const Node<CostType>& goal) {
    auto it = heuristicCache.find(node);
    if (it != heuristicCache.end()) {
        return it->second; // Return cached value
    } 
    CostType heuristicValue = this->heuristicFunction(node, goal);
    heuristicCache[node] = heuristicValue; // Cache the computed value
    return heuristicValue;
}

template class Heuristics<int>;
template class Heuristics<float>;
template class Heuristics<double>;
