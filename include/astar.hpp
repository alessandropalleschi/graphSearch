#ifndef ASTAR_HPP
#define ASTAR_HPP

#include <functional>
#include "searchAlgorithm.hpp"

template <typename CostType>
class AStar : public SearchAlgorithm<CostType> {
private:
    std::function<CostType(const Node<CostType>&, const Node<CostType>&)> heuristicFunction;

public:
    using Path = std::vector<Node<CostType>>;
    AStar(const Graph<CostType>& g, std::function<CostType(const Node<CostType>&, const Node<CostType>&)> heuristic);

    Path search(Node<CostType>& start, Node<CostType>& goal) override;
};

#endif // ASTAR_HPP
