#ifndef ASTAR_HPP
#define ASTAR_HPP

#include <functional>
#include "searchAlgorithms.hpp"

template <typename CostType>
class AStar : public SearchAlgorithm<CostType> {
private:
    std::function<CostType(const Node<CostType>&, const Node<CostType>&)> heuristicFunction;
    std::unordered_map<Node<CostType>, CostType> heuristicCache;

public:
    using Path = std::vector<Node<CostType>>;
    AStar(const Graph<CostType>& graph, std::function<CostType(const Node<CostType>&, const Node<CostType>&)> heuristic);
    CostType getHeuristic(const Node<CostType>& node, const Node<CostType>& goal);
    Path search(Node<CostType>& start, Node<CostType>& goal) override;
};


template <typename CostType>
class Heuristics {
public:
    // Define different heuristics as static member functions
    static CostType simpleHeuristic(const Node<CostType>& current, const Node<CostType>& goal) {
        return abs(goal.ID - current.ID);
    }

    static CostType noHeuristic(const Node<CostType>& current, const Node<CostType>& goal) {
        return CostType(0);
    }

    // Function to get a pointer to a specific heuristic function
    static std::function<CostType(const Node<CostType>&, const Node<CostType>&)> getHeuristicFunction(int heuristicType) {
        switch (heuristicType) {
            case 0:
                return noHeuristic;
            case 1:
                return simpleHeuristic;
            default:
                return noHeuristic; // Default if type not recognized
        }
    }
};

#endif // ASTAR_HPP
