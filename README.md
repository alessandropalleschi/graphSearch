# graphSearch
This project demonstrates a simple implementation in C++ of popular search algorithms. It includes a graph representation, Dijkstra's algorithm, and A* algorithm.

## Table of Contents

- [Features](#features)
- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Usage](#usage)
- [Configuration](#configuration)
- [File Structure](#file-structure)
- [Contributing](#contributing)
- [License](#license)

## Features

- Dijkstra's algorithm for finding the shortest path in a graph.
- A* search algorithm, an informed search algorithm using a heuristic function.
- Flexible graph representation supporting various graph structures.
- Customizable node and edge classes.
- YAML configuration for defining graphs, start and goal nodes, and search algorithms.

## Prerequisites

- C++20
- CMake
- yaml-cpp

## Installation

1. Clone the repository:

   ```bash
   git clone https://github.com/alessandropalleschi/graphSearch.git

2. Navigate to the project directory:
   ```bash
   cd graphSearch
   mkdir build && cd build

3. Build the project using CMake:
   ```bash
    cmake ..
    make
## Usage

Run the executable
   ```bash
   ./graphSearch
```
This will execute the sample main.cpp, demonstrating the usage of Dijkstra's algorithm and A* search algorithm on a graph.

## Configuration

The project supports configuration via a YAML file. The configuration file is located in the config folder. It defines the graph structure, start and goal nodes, and a list of search algorithms to apply.
  ```yaml
totalNodes: 5

startNode: 0
goalNode: 4

edges:
  - source: 0
    destination: 1
    weight: 1
  - source: 1
    destination: 2
    weight: 2
  - source: 2
    destination: 3
    weight: 1
  - source: 3
    destination: 4
    weight: 3
  - source: 0
    destination: 3
    weight: 2

searchAlgorithms:
  - Dijkstra
  - AStar
```
## File Structure

- `src/`: Contains the source files.
  - `main.cpp`: The main program demonstrating the usage of the pathfinding algorithms.
  - `graph.cpp`: Implementation of the graph representation.
  - `dijkstra.cpp`: Implementation of Dijkstra's algorithm.
  - `astar.cpp`: Implementation of the A* algorithm.

- `include/`: Contains header files.
  - `graph.hpp`: Header file for the graph representation.
  - `dijkstra.hpp`: Header file for Dijkstra's algorithm.
  - `astar.hpp`: Header file for the A* algorithm.
  - `searchAlgorithms.hpp`: Header file for the common search algorithms.
- `config/`: Contains yaml file to create grapg.
  - `graph.yaml`
 
### Node
   A Node represents a vertex in the graph and has the following attributes:

   - ID: An integer identifier for the node.
   - name: A string name for the node.
   - cost: A cost associated with the node (used in Dijkstra's algorithm).
   - costToGo: A heuristic cost associated with the node (used in A* algorithm).

   The Node class provides methods to set and get the node's cost, set the heuristic cost (for A*), and compare nodes based on their total cost.

### Edge
   An Edge represents a connection between two nodes in the graph and has the following attributes:

   - source: The source node of the edge.
   - destination: The destination node of the edge.
   - weight: The weight or cost associated with the edge.
### Graph
The Graph class represents the entire graph and uses an adjacency list to store edges for each node. The addEdge method allows you to add an edge between two nodes in the graph.

### Search Algorithms
Both Dijkstra's and A* algorithms are implemented as classes (Dijkstra and AStar, respectively) inheriting from a common base class SearchAlgorithm. They use priority queues for efficient node exploration and maintain a cost map to track the minimum cost to reach each node.


## Contributing
Feel free to contribute by opening issues or pull requests. Contributions are welcome!
## License
This project is licensed under the MIT License - see the LICENSE file for details.



