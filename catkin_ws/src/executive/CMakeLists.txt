cmake_minimum_required(VERSION 2.8.3)
project(executive)

## Add support for C++11, supported in ROS Kinetic and newer
# add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  rosbag
  roscpp
  rospy
  std_msgs
  tf
  roboteq_msgs
  roboteq_driver
  cv_bridge
)
find_package(Boost REQUIRED COMPONENTS date_time regex program_options filesystem)
find_package(BZip2 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES rosbag
  CATKIN_DEPENDS rosbag_storage rosconsole geometry_msgs roscpp rospy std_msgs tf roboteq_driver
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(cust_recorder src/p_recorder.cpp)
target_link_libraries(cust_recorder ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_library(executive src/executive.cpp)
target_link_libraries(executive cust_recorder)

add_executable(${PROJECT_NAME}_node src/executive_node.cpp)
add_executable(hdr_node src/hdr.cpp)

# Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
                      executive
                      ${catkin_LIBRARIES}
)

target_link_libraries(hdr_node
  ${catkin_LIBRARIES}
)

# Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}_node hdr_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
