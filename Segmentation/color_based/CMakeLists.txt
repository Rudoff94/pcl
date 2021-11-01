cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(color_based_segmentation)

find_package(PCL 1.5 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (color_based region_growing_rgb_segmentation.cpp)
target_link_libraries (color_based ${PCL_LIBRARIES})
