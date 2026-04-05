cmake_minimum_required(VERSION 3.0.2)
project(ucar_object_detection)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  cv_bridge
  image_transport
  actionlib
  move_base_msgs
  tf2
  tf2_ros
  std_msgs
 
)

## System dependencies are found with CMake's conventions
find_package(OpenCV REQUIRED)

## Catkin specific configuration
catkin_package(
  CATKIN_DEPENDS roscpp rospy sensor_msgs cv_bridge image_transport actionlib move_base_msgs tf2 tf2_ros std_msgs 
  DEPENDS OpenCV
)

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

## Declare a C++ executable for task_executor
add_executable(task_executor src/task_executor.cpp)
target_link_libraries(task_executor ${catkin_LIBRARIES} ${OpenCV_LIBS})

## Declare a C++ executable for object_selector
add_executable(object_selector src/object_selector.cpp)
target_link_libraries(object_selector ${catkin_LIBRARIES} ${OpenCV_LIBS})

## Mark executable scripts (Python etc.) for installation
catkin_install_python(PROGRAMS
  scripts/ucar_cam.py
  scripts/video.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark other files for installation
install(FILES
  rknn_model/best.rknn
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rknn_model
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
install(TARGETS task_executor
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)