cmake_minimum_required(VERSION 3.0.2)
project(ucar_bringup)

## 查找 catkin 包和依赖
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  detection_msgs
  robot_vision
  ucar_manager
  ucar_object_selector
  ucar_room
  ucar_final
  ucar_traffic
  #ros_yolov5  # ⚠️ 替换为你的 YOLO 推理代码所在的真实包名
)

## 声明 catkin 包
catkin_package(
  CATKIN_DEPENDS
    rospy
  std_msgs
  sensor_msgs
  detection_msgs
  robot_vision
  ucar_manager
  ucar_object_selector
  ucar_room
  ucar_final
  ucar_traffic
  #ros_yolov5 
)

## 没有 C++ 源码也没关系，下面保留标准结构
include_directories(
  ${catkin_INCLUDE_DIRS}
)

# 如果你要以后添加 scripts，可启用这部分（目前不用改）
# catkin_install_python(PROGRAMS
#   scripts/your_script.py
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

