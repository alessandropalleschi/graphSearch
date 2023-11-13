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
   ./graph_search
```
This will execute the sample main.cpp, demonstrating the usage of Dijkstra's algorithm and A* search algorithm on a graph.

## Configuration

The project supports configuration via a YAML file. The configuration file is located in the config folder. It defines the graph structure, start and goal nodes, and a list of search algorithms to apply.
  ```yaml
  nodes: 5
edges:
  - from: 0
    to: 1
    weight: 1
  # Add more edges as needed

start_node: 0
goal_node: 4

search_algorithms:
  - dijkstra
  - astar
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
## Contributing
Feel free to contribute by opening issues or pull requests. Contributions are welcome!
## License
This project is licensed under the MIT License - see the LICENSE file for details.



