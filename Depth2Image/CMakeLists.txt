cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(Depth2Image)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (depth2Image Depth2Image.cpp)
target_link_libraries (depth2Image ${PCL_LIBRARIES})
