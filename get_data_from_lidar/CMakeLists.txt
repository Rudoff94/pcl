cmake_minimum_required(VERSION 3.1.0)

project(convert_frame_from_file)

# Find librealsense2 installed package
find_package(realsense2 REQUIRED)
find_package(PCL 1.2 REQUIRED)
find_package(OpenCV REQUIRED)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Add the application sources to the target
add_executable(${PROJECT_NAME} convert_to_pcd.cpp)

# Link librealsense2 to the target
target_link_libraries(${PROJECT_NAME} ${realsense2_LIBRARY})
target_link_libraries (${PROJECT_NAME} ${PCL_LIBRARIES})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})