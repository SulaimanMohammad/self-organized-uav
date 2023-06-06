cmake_minimum_required(VERSION 3.0.0)
project(self_org_uav VERSION 0.1.0)

cmake_minimum_required(VERSION 3.5)

# set(CMAKE_C_STANDARD 11)

# Add math library
link_libraries(m)

# Create static library target for expansion
add_library(expansion STATIC src/expansion.c include/expansion.h)
add_library(spanning STATIC src/spanning.c include/spanning.h)
add_library(balancing STATIC src/balancing.c include/balancing.h)

# Add include directories
include_directories(include)

# Create executable target for main
add_executable(main main.c)

# Link main target with expansion library
target_link_libraries(main expansion spanning balancing)
