cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(Recognizer)

find_package(PCL 1.2 REQUIRED)
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (recognizer Recognizer.cpp planar_filtering.cpp recognition_features.cpp)
target_link_libraries (recognizer ${PCL_LIBRARIES})
