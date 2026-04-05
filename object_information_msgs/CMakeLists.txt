cmake_minimum_required(VERSION 3.0.2)
project(object_information_msgs)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)  # 启用 C++11 支持

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  std_msgs
  geometry_msgs
  message_generation
)

## 声明消息文件
add_message_files(
  FILES
  Object.msg
)

## 生成消息
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

## 声明包依赖
catkin_package(
  CATKIN_DEPENDS
    message_runtime
    std_msgs
    geometry_msgs
)