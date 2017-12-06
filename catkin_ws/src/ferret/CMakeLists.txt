cmake_minimum_required(VERSION 2.8.3)
project(ferret)

## Add support for C++11, supported in ROS Kinetic and newer
# add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  nav_msgs
  tf2
  geometry_msgs
  laser_assembler
  roboteq_msgs
  rosbag
  pcl_ros
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ferret
#  CATKIN_DEPENDS robot_localization roboteq roscpp rospy sensor_msgs std_msgs urg_node
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
add_executable(${PROJECT_NAME}_tf_broadcaster src/ferret_tf_broadcast.cpp)
target_link_libraries(${PROJECT_NAME}_tf_broadcaster ${catkin_LIBRARIES})
