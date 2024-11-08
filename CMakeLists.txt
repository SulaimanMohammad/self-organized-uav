cmake_minimum_required(VERSION 3.5)
project(self_org_uav VERSION 0.1.0)

# set(CMAKE_C_STANDARD 11)

# Add math library
link_libraries(m)

# Create static library target for expansion, spanning, and balancing
add_library(expansion STATIC src/expansion.c include/expansion.h)
add_library(spanning STATIC src/spanning.c include/spanning.h)
add_library(balancing STATIC src/balancing.c include/balancing.h)

# Add include directories
include_directories(include)

# Create executable target for main
add_executable(main main.c)

# Create executable target for benchmark
add_executable(benchmark_predefined_targets ${CMAKE_BINARY_DIR}/../benchmark/benchmark_predefined_targets.c)
add_executable(benchmark_random_targets ${CMAKE_BINARY_DIR}/../benchmark/benchmark_random_targets.c)

# Link main target with expansion, spanning, and balancing libraries
target_link_libraries(main expansion spanning balancing)
target_link_libraries(benchmark_predefined_targets expansion spanning balancing)
target_link_libraries(benchmark_random_targets expansion spanning balancing)

