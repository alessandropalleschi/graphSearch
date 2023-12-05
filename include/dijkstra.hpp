#ifndef DIJKSTRA_HPP
#define DIJKSTRA_HPP

#include <queue>
#include <unordered_map>
#include "searchAlgorithms.hpp"

template <typename CostType>
class Dijkstra : public SearchAlgorithm<CostType> {
public:
    using Path = std::vector<Node<CostType>>;
    Dijkstra(const Graph<CostType>& g);

    Path search(Node<CostType>& start, Node<CostType>& goal) override;
};


#endif // DIJKSTRA_HPP
