#pragma once

#include <algorithm>
#include <cmath>
#include <functional>
#include <memory>
#include <vector>

#include "graph.hpp"

namespace graphsearch {

/// h(node, goal) -> estimate of the remaining cost. Optimal A* needs it admissible
/// (never overestimates) and, ideally, consistent. Inconsistent heuristics are still handled
/// correctly because A* here re-opens nodes.
template <CostType Cost>
using Heuristic = std::function<Cost(NodeId, NodeId)>;

struct Point {
    double x = 0, y = 0;
};
using Points = std::shared_ptr<const std::vector<Point>>;

template <CostType Cost>
Heuristic<Cost> zeroHeuristic() {
    return [](NodeId, NodeId) { return Cost(0); };
}

/// Straight-line distance. Admissible when every edge weight >= the distance between its ends.
template <CostType Cost>
Heuristic<Cost> euclidean(Points p) {
    return [p](NodeId a, NodeId b) {
        return static_cast<Cost>(std::hypot((*p)[a].x - (*p)[b].x, (*p)[a].y - (*p)[b].y));
    };
}

/// |dx| + |dy|. Admissible on 4-connected unit-cost grids.
template <CostType Cost>
Heuristic<Cost> manhattan(Points p) {
    return [p](NodeId a, NodeId b) {
        return static_cast<Cost>(std::abs((*p)[a].x - (*p)[b].x) + std::abs((*p)[a].y - (*p)[b].y));
    };
}

/// Octile distance (8-connected, diagonal cost sqrt(2), straight cost 1).
template <CostType Cost>
Heuristic<Cost> octile(Points p) {
    return [p](NodeId a, NodeId b) {
        const double dx = std::abs((*p)[a].x - (*p)[b].x), dy = std::abs((*p)[a].y - (*p)[b].y);
        return static_cast<Cost>(dx + dy + (std::sqrt(2.0) - 2.0) * std::min(dx, dy));
    };
}

/// Octile distance for a row-major grid of the given width, derived from node ids (no table).
template <CostType Cost>
Heuristic<Cost> gridOctile(std::size_t width) {
    return [width](NodeId a, NodeId b) {
        const double dx = std::abs(double(a % width) - double(b % width));
        const double dy = std::abs(double(a / width) - double(b / width));
        return static_cast<Cost>(dx + dy + (std::sqrt(2.0) - 2.0) * std::min(dx, dy));
    };
}

}  // namespace graphsearch
