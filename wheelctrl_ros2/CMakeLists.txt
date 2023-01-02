cmake_minimum_required(VERSION 3.8)
project(wheelctrl_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rogilink2_interfaces REQUIRED)
find_package(md_lib REQUIRED)


add_executable(wheelctrl_ros2 
src/wheelctrl_ros2.cpp 
src/omni_2w.cpp
src/omni_3w.cpp
src/omni_4w.cpp
src/steering.cpp
)

ament_target_dependencies(
  wheelctrl_ros2 
  rclcpp 
  std_msgs 
  nav_msgs 
  geometry_msgs 
  tf2 
  tf2_ros 
  rogilink2_interfaces 
  md_lib)

install(TARGETS
  wheelctrl_ros2 
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY 
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
