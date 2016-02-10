cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(moment_of_inertia2)

find_package(PCL 1.7.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (moment_of_inertia2 moment_of_inertia2.cpp boundingBox.cpp)
target_link_libraries (moment_of_inertia2 ${PCL_LIBRARIES})

