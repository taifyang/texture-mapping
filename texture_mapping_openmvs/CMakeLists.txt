cmake_minimum_required(VERSION 3.0)
project(texture_mapping_openmvs)
add_definitions(-std=c++14)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

find_package(Boost COMPONENTS iostreams program_options system serialization REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

find_package(CGAL REQUIRED)
include_directories(${CGAL_INCLUDE_DIRS})

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(OpenMVS REQUIRED)
include_directories(${OpenMVS_INCLUDE_DIRS})
add_definitions(${OpenMVS_DEFINITIONS})

add_executable(texture_mapping_openmvs texture_mapping_openmvs.cpp)
target_link_libraries(texture_mapping_openmvs ${Eigen3_LIBRARIES})
target_link_libraries(texture_mapping_openmvs ${Boost_LIBRARIES})
target_link_libraries(texture_mapping_openmvs ${CGAL_LIBRARIES})
target_link_libraries(texture_mapping_openmvs ${PCL_LIBRARIES})
target_link_libraries(texture_mapping_openmvs OpenMVS::MVS)