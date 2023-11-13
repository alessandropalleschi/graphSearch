#include <iostream>
#include <fstream>
#include "Graph.hpp"
#include "Dijkstra.hpp"
#include "AStar.hpp"
#include "yaml-cpp/yaml.h"

int main() {
    // Read graph information from YAML file in the config folder
    YAML::Node config = YAML::LoadFile("config/graph.yaml");

    const int totalNodes = config["totalNodes"].as<int>();
    const int startNodeID = config["startNode"].as<int>();
    const int goalNodeID = config["goalNode"].as<int>();
    const YAML::Node edges = config["edges"];
    const YAML::Node searchAlgorithms = config["searchAlgorithms"];

    // Create nodes
    Graph<int> intGraph;
    std::vector<Node<int>> nodes;

    for (int i = 0; i < totalNodes; ++i) {
        nodes.emplace_back(i, "Node" + std::to_string(i));
    }

    // Add edges from YAML configuration
    for (const auto& edge : edges) {
        int sourceID = edge["source"].as<int>();
        int destID = edge["destination"].as<int>();
        int weight = edge["weight"].as<int>();

        intGraph.addEdge(nodes[sourceID], nodes[destID], weight);
    }

    // Get start and goal nodes
    Node<int> startNode = nodes[startNodeID];
    Node<int> goalNode = nodes[goalNodeID];

    // Perform searches based on the specified algorithms
    for (const auto& algorithm : searchAlgorithms) {
        std::string algorithmType = algorithm["type"].as<std::string>();
    
        if (algorithmType == "Dijkstra") {
            Dijkstra<int> intDijkstra(intGraph);
            std::cout << "\nDijkstra's Path:" << std::endl;
            auto dijkstraPath = intDijkstra.search(startNode, goalNode);
            // Output or handle the result as needed
        } else if (algorithmType == "AStar") {
            int heuristicType = algorithm["heuristic"].as<int>();
            AStar<int> intAStar(intGraph, Heuristics<int>::getHeuristicFunction(heuristicType));
            std::cout << "\nA* Path:" << std::endl;
            auto aStarPath = intAStar.search(startNode, goalNode);
            // Output or handle the result as needed
        } else {
            std::cerr << "Unknown search algorithm: " << algorithmType << std::endl;
        }
}

    return 0;
}
