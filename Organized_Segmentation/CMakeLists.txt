cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(organized_segmentation)

find_package(PCL 1.4 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (organized_segmentation organized_segmentation.cpp)
target_link_libraries (organized_segmentation ${PCL_LIBRARIES})

