cmake_minimum_required(VERSION 3.15)
project(MyProject)

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(test main.cpp)
target_link_libraries(test ${PCL_LIBRARIES})

