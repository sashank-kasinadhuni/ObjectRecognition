cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(multi_plane_segmentation)

find_package(PCL 1.4 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (multi_plane_segmentation multi_plane_segmentation.cpp)
target_link_libraries (multi_plane_segmentation ${PCL_LIBRARIES})

