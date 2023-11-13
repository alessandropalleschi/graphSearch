cmake_minimum_required(VERSION 3.12)

project(graphSearch)

# Set C++ standard to C++20
set(CMAKE_CXX_STANDARD 20)

find_package(yaml-cpp REQUIRED)

# Add the src and include directories
include_directories(include)

# Add executable
add_executable(graphSearch src/main.cpp)

# Add source files for the library
add_library(graphSearchLib
    src/graph.cpp
    src/dijkstra.cpp
    src/astar.cpp
)

# Link the library with the executable
target_link_libraries(graphSearch PRIVATE graphSearchLib)
