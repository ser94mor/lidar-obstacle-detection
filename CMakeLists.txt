cmake_minimum_required(VERSION 3.14)
project(lidar_obstacle_detection)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -pedantic -Wextra")

include_directories(third_parties/eigen-3.3.7)
include_directories(third_parties/catch2-2.7.0)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (lidar_obstacle_detection src/main.cpp src/kd_tree.hpp)

add_library(Catch INTERFACE)
add_executable(lidar_obstacle_detection_test
        test/kd_tree_test.cpp
        test/test.cpp)
target_link_libraries(lidar_obstacle_detection_test Catch)

enable_testing()
add_test( NAME lidar_obstacle_detection_test COMMAND lidar_obstacle_detection_test )
