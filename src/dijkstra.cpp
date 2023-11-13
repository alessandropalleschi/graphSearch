#include "Dijkstra.hpp"

// Constructor for Dijkstra class
template <typename CostType>
Dijkstra<CostType>::Dijkstra(const Graph<CostType>& g) : SearchAlgorithm<CostType>(g) {}

// Dijkstra's search algorithm implementation
template <typename CostType>
typename Dijkstra<CostType>::Path Dijkstra<CostType>::search(Node<CostType>& start, Node<CostType>& goal) {
    Path path;
    // Priority queue to store nodes to visit, using the custom Compare class for ordering
    std::priority_queue<Node<CostType>, std::vector<Node<CostType>>, Compare<CostType>> toVisit;
    // Map to store the cost of reaching each node
    std::unordered_map<Node<CostType>, CostType> costMap;

    // Initialize start node's cost and add it to the priority queue
    start.setNodeCost(CostType(0));
    toVisit.push(start);
    costMap[start] = CostType(0);

    // Main loop for Dijkstra's search
    while (!toVisit.empty()) {
        // Get the node with the lowest cost from the priority queue
        Node<CostType> visited_node = toVisit.top();
        toVisit.pop();

        // Check if the node has already been visited with a lower cost
        if (costMap.find(visited_node) != costMap.end() && visited_node.cost > costMap[visited_node]) {
            continue;
        }

        // Add the visited node to the path
        path.push_back(visited_node);

        // Check if the goal has been reached
        if (visited_node == goal) {
            return path;
        }

        // Explore neighbors of the current node
        for (const auto& edge : this->getNeighbours(visited_node)) {
            Node<CostType> adjacent_node = edge.destination;
            CostType newCost = costMap[visited_node] + edge.weight;

            // If the new cost is smaller, update the cost and add to the priority queue
            if (costMap.find(adjacent_node) == costMap.end() || newCost < costMap[adjacent_node]) {
                costMap[adjacent_node] = newCost;
                adjacent_node.setNodeCost(newCost);
                toVisit.push(adjacent_node);
            }
        }
    }

    // No path found
    return {};
}

// Explicit instantiation for int
template class Dijkstra<int>;
