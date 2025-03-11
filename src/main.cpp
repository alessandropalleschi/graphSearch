#include <iostream>
#include <fstream>
#include "graph.hpp"
#include "dijkstra.hpp"
#include "astar.hpp"
#include "yaml-cpp/yaml.h"

void displayPath(const std::vector<Node<int>>& path) {
    if (path.empty()) {
        std::cout << "No path found." << std::endl;
        return;
    }
    std::cout << "Path: ";
    for (const auto& node : path) {
        std::cout << node.ID << " ";
    }
    std::cout << std::endl;
}

int main() {
    try {
        YAML::Node config = YAML::LoadFile("config/graph.yaml");

        const int totalNodes = config["totalNodes"].as<int>();
        const int startNodeID = config["startNode"].as<int>();
        const int goalNodeID = config["goalNode"].as<int>();
        const YAML::Node edges = config["edges"];
        const YAML::Node searchAlgorithms = config["searchAlgorithms"];

        Graph<int> intGraph;
        std::vector<Node<int>> nodes(totalNodes);

        for (int i = 0; i < totalNodes; ++i) {
            nodes[i] = Node<int>(i, "Node" + std::to_string(i));
        }

        for (const auto& edge : edges) {
            int sourceID = edge["source"].as<int>();
            int destID = edge["destination"].as<int>();
            int weight = edge["weight"].as<int>();
            intGraph.addEdge(nodes[sourceID], nodes[destID], weight);
        }

        Node<int>& startNode = nodes[startNodeID];
        Node<int>& goalNode = nodes[goalNodeID];

        for (const auto& algorithm : searchAlgorithms) {
            std::string algorithmType = algorithm["type"].as<std::string>();

            if (algorithmType == "Dijkstra") {
                Dijkstra<int> intDijkstra(intGraph);
                std::cout << "\nDijkstra's Path:" << std::endl;
                displayPath(intDijkstra.search(startNode, goalNode));
            } else if (algorithmType == "AStar") {
                int heuristicType = algorithm["heuristics"].as<int>();
                AStar<int> intAStar(intGraph, Heuristics<int>::getHeuristicFunction(heuristicType));
                std::cout << "\nA* Path:" << std::endl;
                displayPath(intAStar.search(startNode, goalNode));
            } else {
                std::cerr << "Unknown search algorithm: " << algorithmType << std::endl;
            }
        }

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
    }

    return 0;
}
