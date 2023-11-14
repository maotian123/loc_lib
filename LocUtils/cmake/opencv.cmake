include_directories(include ${catkin_INCLUDE_DIRS})
find_package(OpenCV REQUIRED)
include_directories(include ${OpenCV_INCLUDE_DIRS})
# list(APPEND ALL_TARGET_LIBRARIES ${OpenCV_LIB_DIR})
list(APPEND ALL_TARGET_LIBRARIES ${OpenCV_LIBRARIES}) 
# 我的环境此处应该是${OpenCV_LIBRARIES}，否则找不到opecv的libs，设置不到locutils的config.cmake里面去
