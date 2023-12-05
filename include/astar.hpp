#ifndef ASTAR_HPP
#define ASTAR_HPP

#include <functional>
#include "searchAlgorithms.hpp"

template <typename CostType>
class AStar : public SearchAlgorithm<CostType> {
private:
    std::function<CostType(const Node<CostType>&, const Node<CostType>&)> heuristicFunction;

public:
    using Path = std::vector<Node<CostType>>;
    AStar(const Graph<CostType>& g, std::function<CostType(const Node<CostType>&, const Node<CostType>&)> heuristic);
    CostType getHeuristic(const Node<CostType>& node, const Node<CostType>& goal);
    Path search(Node<CostType>& start, Node<CostType>& goal) override;
    std::unordered_map<Node<CostType>,CostType> heuristicCache;
};


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


#endif // ASTAR_HPP
