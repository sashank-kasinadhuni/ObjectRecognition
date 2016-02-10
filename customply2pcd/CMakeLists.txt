cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(customply2pcd)

find_package(PCL 1.5 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (customtxt2pcd customtxt2pcd.cpp)
target_link_libraries (customtxt2pcd ${PCL_LIBRARIES})

