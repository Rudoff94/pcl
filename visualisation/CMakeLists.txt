cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(cloud_visualizer)

set (CMAKE_CXX_STANDARD 17)
find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (cloud_visualizer_rgba visualize_pcd.cpp)
target_link_libraries (cloud_visualizer_rgba ${PCL_LIBRARIES})
