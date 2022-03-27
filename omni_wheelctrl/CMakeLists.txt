cmake_minimum_required(VERSION 3.0.2)
project(omni_wheelctrl)

## Compile as C++11, supported in ROS Kinetic and newer
set(CMAKE_CXX_STANDARD 14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  tf
  rogi_link_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES swerve_wheelctrl
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_subdirectory(src/converter)

add_executable(omni_wheelctrl src/main.cpp)
target_link_libraries(omni_wheelctrl ${catkin_LIBRARIES} converter_omni)
add_dependencies(omni_wheelctrl ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})