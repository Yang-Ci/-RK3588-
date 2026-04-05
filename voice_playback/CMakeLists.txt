cmake_minimum_required(VERSION 3.0.2)
project(voice_playback)

# 查找 catkin 依赖
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

# 声明 catkin 包的运行时依赖
catkin_package(
  CATKIN_DEPENDS rospy std_msgs
)

# 包含头文件目录
include_directories(
  ${catkin_INCLUDE_DIRS}
)

# 安装 Python 节点到指定目录
catkin_install_python(PROGRAMS src/voice_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
