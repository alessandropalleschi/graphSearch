cmake_minimum_required(VERSION 3.14)
project(graphSearch VERSION 3.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release)
endif()

option(GRAPHSEARCH_BUILD_APP "Build the YAML-driven CLI (needs yaml-cpp)" ON)
option(GRAPHSEARCH_BUILD_TESTS "Build tests" ON)
option(GRAPHSEARCH_BUILD_BENCHMARKS "Build benchmarks" ON)
option(GRAPHSEARCH_WERROR "Treat warnings as errors" OFF)
option(GRAPHSEARCH_SANITIZE "Build with AddressSanitizer + UBSan" OFF)

if(GRAPHSEARCH_SANITIZE)
  add_compile_options(-fsanitize=address,undefined -fno-omit-frame-pointer)
  add_link_options(-fsanitize=address,undefined)
endif()

# ---- Header-only core library -------------------------------------------------------------
add_library(graphsearch INTERFACE)
add_library(graphsearch::graphsearch ALIAS graphsearch)
target_include_directories(graphsearch INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(graphsearch INTERFACE cxx_std_20)

function(graphsearch_warnings target)
  if(MSVC)
    target_compile_options(${target} PRIVATE /W4 $<$<BOOL:${GRAPHSEARCH_WERROR}>:/WX>)
  else()
    target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic
      $<$<BOOL:${GRAPHSEARCH_WERROR}>:-Werror>)
  endif()
endfunction()

# ---- CLI + YAML loader --------------------------------------------------------------------
if(GRAPHSEARCH_BUILD_APP)
  find_package(yaml-cpp REQUIRED)
  add_library(graphsearch_config STATIC src/config.cpp)
  target_include_directories(graphsearch_config PUBLIC src)
  target_link_libraries(graphsearch_config PUBLIC graphsearch yaml-cpp)
  graphsearch_warnings(graphsearch_config)

  add_executable(graphSearch src/main.cpp)
  target_link_libraries(graphSearch PRIVATE graphsearch_config)
  graphsearch_warnings(graphSearch)
endif()

add_executable(grid_astar examples/grid_astar.cpp)
target_link_libraries(grid_astar PRIVATE graphsearch)
add_executable(visualize_grid examples/visualize_grid.cpp)
target_link_libraries(visualize_grid PRIVATE graphsearch)
graphsearch_warnings(visualize_grid)

# ---- Tests --------------------------------------------------------------------------------
if(GRAPHSEARCH_BUILD_TESTS)
  enable_testing()
  add_executable(test_search tests/test_search.cpp)
  target_link_libraries(test_search PRIVATE graphsearch)
  graphsearch_warnings(test_search)
  add_test(NAME search COMMAND test_search)
  if(GRAPHSEARCH_BUILD_APP)
    add_executable(test_config tests/test_config.cpp)
    target_link_libraries(test_config PRIVATE graphsearch_config)
    graphsearch_warnings(test_config)
    add_test(NAME config COMMAND test_config)
    add_test(NAME cli COMMAND graphSearch ${CMAKE_CURRENT_SOURCE_DIR}/config/graph.yaml)
  endif()
endif()

# ---- Benchmarks ---------------------------------------------------------------------------
if(GRAPHSEARCH_BUILD_BENCHMARKS)
  add_executable(bench benchmarks/bench.cpp)
  target_link_libraries(bench PRIVATE graphsearch)
  graphsearch_warnings(bench)
endif()

# ---- Install / find_package(graphsearch) --------------------------------------------------
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)
install(TARGETS graphsearch EXPORT graphsearchTargets)
install(DIRECTORY include/graphsearch DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(EXPORT graphsearchTargets NAMESPACE graphsearch:: DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/graphsearch)
write_basic_package_version_file(${CMAKE_CURRENT_BINARY_DIR}/graphsearchConfigVersion.cmake
  COMPATIBILITY SameMajorVersion ARCH_INDEPENDENT)
configure_package_config_file(cmake/graphsearchConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/graphsearchConfig.cmake
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/graphsearch)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/graphsearchConfig.cmake
              ${CMAKE_CURRENT_BINARY_DIR}/graphsearchConfigVersion.cmake
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/graphsearch)
