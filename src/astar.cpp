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

        if (costMap.find(visited_node) != costMap.end() && visited_node.cost > costMap[visited_node]) {
            continue;
        }

        path.push_back(visited_node);

        if (visited_node == goal) {
            return path;
        }

        for (const auto& edge : this->getNeighbours(visited_node)) {
            Node<CostType> adjacent_node = edge.destination;
            CostType newCost = costMap[visited_node] + edge.weight + getHeuristic(adjacent_node, goal);

            if (costMap.find(adjacent_node) == costMap.end() || newCost < costMap[adjacent_node]) {
                costMap[adjacent_node] = newCost;
                adjacent_node.setNodeCost(newCost, getHeuristic(adjacent_node, goal));
                toVisit.push(adjacent_node);
            }
        }
    }

    return {};
}


template <typename CostType>
AStar<CostType>::CostType getHeuristic(const Node<CostType>& node, const Node<CostType>& goal) {
    auto it = heuristicCache.find(node);
    if (it != heuristicCache.end()) {
        return it->second; // Return cached value
    } else {
        CostType heuristicValue = this->heuristicFunction(node, goal);
        heuristicCache[node] = heuristicValue; // Cache the computed value
        return heuristicValue;
    }
}

// Explicit instantiation for int
template class AStar<int>;

template <typename CostType>
class Heuristics {
public:
    // Define different heuristics as static member functions
    static CostType simpleHeuristic(const Node<CostType>& current, const Node<CostType>& goal) {
        
        return abs(goal.ID-current.ID);
    }
    /*
    static CostType customHeuristic(const Node<CostType>& current, const Node<CostType>& goal) {
        // Implement another heuristic
       pass;
    }
    */
    static CostType noHeuristic(const Node<CostType>& current, const Node<CostType>& goal) {
        // Implement another heuristic
       return CostType(0);
    }

    // Add more heuristics as needed

    // Function to get a pointer to a specific heuristic function
    static std::function<CostType(const Node<CostType>&, const Node<CostType>&)> getHeuristicFunction(int heuristicType) {
        switch (heuristicType) {
            case 0:
                return noHeuristic;
            case 1:
                return simpleHeuristic;
            // Add more cases for other heuristics
            default:
                // Default to a simple heuristic if the specified type is not recognized
                return noHeuristic;
        }
    }
};
// Explicit instantiation for int
template class Heuristics<int>;
