#ifndef DIJKSTRA_HPP
#define DIJKSTRA_HPP

#include <queue>
#include <unordered_map>
#include "searchAlgorithms.hpp"

template <typename CostType>
class Dijkstra : public SearchAlgorithm<CostType> {
private:
    std::unordered_map<Node<CostType>, CostType> costMap; // Added for improved tracking

public:
    using Path = std::vector<Node<CostType>>;
    Dijkstra(const Graph<CostType>& graph);

    Path search(Node<CostType>& start, Node<CostType>& goal) override;
};

#endif // DIJKSTRA_HPP
