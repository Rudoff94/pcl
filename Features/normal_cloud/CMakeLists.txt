cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(normal_cloud)

find_package(PCL 1.2 REQUIRED)
set (CMAKE_CXX_STANDARD 17)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(normal_1 normal_cloud.cpp)
target_link_libraries(normal_1 ${PCL_LIBRARIES})
