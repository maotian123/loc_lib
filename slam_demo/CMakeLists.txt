cmake_minimum_required(VERSION 3.10.2)
project(slam_demo)

SET(CMAKE_CXX_STANDARD 17)
SET(CMAKE_BUILD_TYPE "Release")
SET(CMAKE_CXX_FLAGS "-std=c++17 -O3 -Wall")
SET(THIRD_PATRY_LIBS "")
SET(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 魏志飞workspace特有
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "/usr/share/cmake/geographiclib")

SET(param_source 
    src/params/slam_params.cpp
)

SET(lio_mapping_source
    src/mapping/3d/lio_mapping_flow.cpp
)

SET(lio_matching_source
    src/matching/3d/lio_matching_flow.cpp
)

add_library(2d_slam_lib SHARED
            ${2d_slam_source}  
            ${param_source}
)

add_library(lio_mapping_lib SHARED
            ${lio_mapping_source}
)

add_library(lio_matching_lib SHARED
            ${lio_matching_source}
)

find_package(LocUtils REQUIRED)

message(LocUtils_INCLUDE_DIRS ${LocUtils_INCLUDE_DIRS})
include_directories(
    include
    ${LocUtils_INCLUDE_DIRS}
)

# add_executable(fpga_node src/apps/fpga_node.cpp)
# target_link_libraries(fpga_node  glog gflags ${LocUtils_LIBRARIES})

# add_executable(test_node src/apps/test_node.cpp)
# target_link_libraries(test_node   2d_slam_lib glog gflags ${LocUtils_LIBRARIES})

add_executable(lio_mapping_node src/apps/lio_mapping_node.cpp)
target_link_libraries(lio_mapping_node   lio_mapping_lib glog gflags ${LocUtils_LIBRARIES})

add_executable(lio_matching_node src/apps/lio_matching_node.cpp)
target_link_libraries(lio_matching_node   lio_matching_lib glog gflags ${LocUtils_LIBRARIES})
