cmake_minimum_required(VERSION 3.12)

project(graphSearch)

# Set C++ standard to C++20
set(CMAKE_CXX_STANDARD 20)

find_package(yaml-cpp REQUIRED)

# Add the src and include directories
include_directories(include)

file(GLOB_RECURSE SRC_FILES
    src/*.cpp
)


# Add executable
add_executable(graphSearch ${SRC_FILES})

# Link the library with the executable
target_link_libraries(graphSearch yaml-cpp)
