find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    geometry_msgs
    sensor_msgs
    visualization_msgs
    pcl_ros
    tf
    eigen_conversions
)
include_directories(include ${catkin_INCLUDE_DIRS})
list(APPEND ALL_TARGET_LIBRARIES ${catkin_LIBRARIES})
list(APPEND INCLUDE_DIR_LIST ${catkin_INCLUDE_DIRS})
