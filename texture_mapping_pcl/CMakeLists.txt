cmake_minimum_required(VERSION 3.0)
project(texture_mapping_pcl)

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(texture_mapping_pcl texture_mapping_pcl.cpp)
target_link_libraries(texture_mapping_pcl${PCL_LIBRARIES})