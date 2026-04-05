cmake_minimum_required(VERSION 3.0.2)
project(send_goals)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  actionlib
  move_base_msgs
  roscpp
  std_msgs
  tf2
  tf2_ros
  geometry_msgs
  object_information_msgs
)
find_package(Eigen3 REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS
    actionlib
    move_base_msgs
    roscpp
    std_msgs
    tf2
    tf2_ros
    geometry_msgs
    object_information_msgs
  DEPENDS EIGEN3
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

## Declare all C++ executables
add_executable(${PROJECT_NAME}_node src/send_goals_node.cpp)
add_executable(${PROJECT_NAME}_b src/send_goals_b.cpp)
add_executable(${PROJECT_NAME}_c src/send_goals_c.cpp)
add_executable(${PROJECT_NAME}_d src/send_goals_d.cpp)
add_executable(${PROJECT_NAME}_dingyue src/dingyue.cpp)
add_executable(${PROJECT_NAME}_getS src/getS.cpp)
add_executable(${PROJECT_NAME}_getB src/getB.cpp)
add_executable(${PROJECT_NAME}_getV src/getV.cpp)
add_executable(${PROJECT_NAME}_rotationb src/rotationb.cpp)
add_executable(${PROJECT_NAME}_rotations src/rotations.cpp)
add_executable(${PROJECT_NAME}_rotationv src/rotationv.cpp)





## Link all executables to catkin libraries
## (This should include tf2_ros since it's in catkin_LIBRARIES)
foreach(EXECUTABLE 
  ${PROJECT_NAME}_node 
  ${PROJECT_NAME}_b 
  ${PROJECT_NAME}_c 
  ${PROJECT_NAME}_d 
  ${PROJECT_NAME}_dingyue 
  ${PROJECT_NAME}_getS 
  ${PROJECT_NAME}_getB 
  ${PROJECT_NAME}_getV 
  ${PROJECT_NAME}_rotationb 
  ${PROJECT_NAME}_rotations 
  ${PROJECT_NAME}_rotationv
)
  target_link_libraries(${EXECUTABLE}
    ${catkin_LIBRARIES}
  )
endforeach()