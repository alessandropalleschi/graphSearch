#include "astar.hpp"

// Constructor for AStar class
template <typename CostType>
AStar<CostType>::AStar(const Graph<CostType>& g, std::function<CostType(const Node<CostType>&, const Node<CostType>&)> heuristic)
    : SearchAlgorithm<CostType>(g), heuristicFunction(std::move(heuristic)) {}

// A* search algorithm implementation
template <typename CostType>
typename AStar<CostType>::Path AStar<CostType>::search(Node<CostType>& start, Node<CostType>& goal) {
    Path path;
    std::priority_queue<Node<CostType>, std::vector<Node<CostType>>, Compare<CostType>> toVisit;
    std::unordered_map<Node<CostType>, CostType> costMap;

    start.setNodeCost(CostType(0));
    toVisit.push(start);
    costMap[start] = CostType(0);

    while (!toVisit.empty()) {
        Node<CostType> visited_node = toVisit.top();
        toVisit.pop();

        if (costMap.find(visited_node) != costMap.end() && visited_node.costToGo > costMap[visited_node]) {
            continue;
        }

        path.push_back(visited_node);

        if (visited_node == goal) {
            return path;
        }

        for (const auto& edge : this->getNeighbours(visited_node)) {
            Node<CostType> adjacent_node = edge.destination;
            CostType newCost = costMap[visited_node] + edge.weight;

            if (costMap.find(adjacent_node) == costMap.end() || newCost < costMap[adjacent_node]) {
                costMap[adjacent_node] = newCost;
                adjacent_node.setNodeCost(newCost, this->getHeuristic(adjacent_node, goal));
                toVisit.push(adjacent_node);
            }
        }
    }

    return {};
}

template class AStar<int>;
template class AStar<float>;
template class AStar<double>;

template <typename CostType>
CostType AStar<CostType>::getHeuristic(const Node<CostType>& node, const Node<CostType>& goal) {
    auto it = heuristicCache.find(node);
    if (it != heuristicCache.end()) {
        return it->second; // Return cached value
    } else {
        CostType heuristicValue = this->heuristicFunction(node, goal);
        heuristicCache[node] = heuristicValue; // Cache the computed value
        return heuristicValue;
    }
}

template class Heuristics<int>;
template class Heuristics<float>;
template class Heuristics<double>;

