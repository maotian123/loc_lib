cmake_minimum_required(VERSION 3.10.2)
project(LocUtils)

SET(CMAKE_CXX_STANDARD 17)
SET(CMAKE_BUILD_TYPE "Release")
SET(CMAKE_CXX_FLAGS "-std=c++17 -O3 -Wall")
# 只编译通信相关的

SET(BUILD_ALGORITHM ON)

SET(ALL_TARGET_LIBRARIES "")
SET(INCLUDE_DIR_LIST "")

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/libs)

include (cmake/common.cmake)


FIND_INCLUDE_DIR(INCLUDE_DIR_LIST ${CMAKE_CURRENT_SOURCE_DIR})

include (cmake/catkin.cmake)
include (cmake/yaml.cmake)
include (cmake/geographic.cmake)

if(BUILD_ALGORITHM)
  include(cmake/sophus.cmake)
  # include(cmake/g2o.cmake)
  include(cmake/opencv.cmake)
  # include(cmake/CSPARSE.cmake)
endif()

SET(SENSOR_DATA
  src/sensor_data/gnss_data.cpp
)
SET(PUB_SUB_SRC
  src/publisher/cloud_publisher.cpp
  src/publisher/imu_publisher.cpp
  src/publisher/scan_publisher.cpp
  src/publisher/odometry_publisher.cpp
  src/publisher/maker_publisher.cpp

  src/subscriber/cloud_subscriber.cpp
  src/subscriber/imu_subscriber.cpp
  src/subscriber/scan_subscriber.cpp
  src/subscriber/odometry_subscriber.cpp
  # src/subscriber/scan_subscriber.cpp
  src/subscriber/gnss_subscriber.cpp

  src/tf_lisener/tf_lisener.cpp
)

SET(COMMON_SRC
  src/common/common.cpp
  src/common/pose_data.cpp
  src/common/point_cloud_utils.cpp
  src/common/gnss_utils.cpp
  src/common/imu.cpp
)

SET(CLOUD_FILTER_SRC
  src/model/cloud_filter/no_filter.cpp
  src/model/cloud_filter/voxel_filter.cpp
  src/model/cloud_filter/box_filter.cpp

  src/model/feature_extract/loam_feature_extract.cpp
  src/model/loop_closure/scan_context.cpp
)

SET(2D_MAPPING_SRC
  src/mapping_2d/loop_closing.cpp
  src/mapping_2d/mapping_2d.cpp
  src/mapping_2d/multi_resolution_likelihood_field.cpp
  src/mapping_2d/submap.cpp

  src/model/likelihoodField/likelihood_filed.cpp
  src/model/occupancy_grid/occupancy_map.cpp
)

SET(3D_FRONT_END_SRC
  # src/model/registration/3d/icp_registration.cpp
  src/model/matching/3d/icp/icp_registration.cpp
  src/model/matching/3d/ndt/ndt_registration.cpp
  src/model/matching/3d/loam/loam_registration.cpp
  src/model/search_point/kdtree/kdtree.cpp
  src/model/search_point/bfnn/bfnn.cpp
  src/model/integration/imu_integration.cpp
)

SET(MAPPING_SRC
  ${CLOUD_FILTER_SRC}
  src/model/sync/measure_sync.cpp
  src/slam/3d/lio.cpp
)

SET(MATCHING_SRC
  ${CLOUD_FILTER_SRC}
  src/model/sync/measure_sync.cpp
  src/slam/3d/loc.cpp
)

SET(SENSOR_FUSION_SRC
  # src/model/registration/3d/icp_registration.cpp
  # src/model/kdtree/kdtree.cpp
  src/model/static_init/static_imu_init.cpp
  src/model/eskf/eskf.cpp
)

SET(TOOLS_SRC
  # src/model/registration/3d/icp_registration.cpp
  # src/model/kdtree/kdtree.cpp
  # src/model/reflactor/reflactor_proc.cpp
  src/tools/file_manager.cpp
  src/tools/save_pose.cpp
)

SET(LIO_SRC
  src/slam/3d/lio.cpp
)

message(INCLUDE_DIR_LIST ${INCLUDE_DIR_LIST})

if(BUILD_ALGORITHM)
  message("BUILD_ALGORITHM")
  add_library(LocUtils ${3D_FRONT_END_SRC} ${SENSOR_FUSION_SRC}
                       ${PUB_SUB_SRC} ${COMMON_SRC} ${MAPPING_SRC} ${MATCHING_SRC} ${TOOLS_SRC} ${SENSOR_DATA})
  target_link_libraries(LocUtils PUBLIC ${ALL_TARGET_LIBRARIES} cholmod 
                        glog gflags)
else()
  message("NOT BUILD_ALGORITHM")
  add_library(LocUtils ${PUB_SUB_SRC} ${COMMON_SRC} ${TOOLS_SRC} ${SENSOR_DATA})
  target_link_libraries(LocUtils PUBLIC ${ALL_TARGET_LIBRARIES} glog gflags)
endif()

# Set installation rules
install(TARGETS LocUtils
        EXPORT locutils-targets
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
)

install(DIRECTORY include/ DESTINATION include)

install(EXPORT locutils-targets
        NAMESPACE locutils::
        FILE locutils-targets.cmake
        DESTINATION share/LocUtils
)
# Generate package configuration file
configure_file(
        ${CMAKE_CURRENT_SOURCE_DIR}/cmake/locutils-config.cmake.in
        ${CMAKE_CURRENT_BINARY_DIR}/locutils-config.cmake @ONLY)

file(WRITE ${CMAKE_CURRENT_SOURCE_DIR}/cmake/locutils-config.cmake.in 
    "\@PACKAGE_INIT\@
    find_package(GeographicLib REQUIRED)
    find_package(yaml-cpp REQUIRED)
    get_filename_component (_dir \"\${CMAKE_CURRENT_LIST_FILE}\" PATH)
    get_filename_component (_prefix \"\${_dir}/../..\" ABSOLUTE)
    message(\"Find LocUtils Success\")
    message(\"CMAKE_CURRENT_LIST_FILE : \" \${CMAKE_CURRENT_LIST_FILE})
    # Import the targets.
    include (\"\${_prefix}/share/LocUtils/locutils-targets.cmake\")
    set(LocUtils_INCLUDE_DIRS \"\")
    list(APPEND LocUtils_INCLUDE_DIRS \"${INCLUDE_DIR_LIST}\")
    set(LocUtils_LIBRARIES \"\${_prefix}/lib/libLocUtils.so\")
    list(APPEND LocUtils_LIBRARIES \"${ALL_TARGET_LIBRARIES}\")  

    message(\"LocUtils_INCLUDE_DIRS : \" \${LocUtils_INCLUDE_DIRS})
    message(\"LocUtils_LIBRARIES : \" \${LocUtils_LIBRARIES})

    #导出变量
    set(LocUtils_FOUND TRUE)
")
     
# Install package configuration files
install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/locutils-config.cmake"
    DESTINATION share/LocUtils
)