cmake_minimum_required(VERSION 3.0.2)
project(mecanum_odom_publisher)
## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  nav_msgs
  tf
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES swerve_odom_publisher
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
add_executable(mecanum_odom_publisher src/mecanum_odom_publisher.cpp)
target_link_libraries(mecanum_odom_publisher ${catkin_LIBRARIES})
add_dependencies(mecanum_odom_publisher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})