cmake_minimum_required(VERSION 3.8)
project(robot_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(custom_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosidl_default_runtime REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)

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

add_executable(wall_finder_server src/wall_finder_server.cpp)
add_executable(wall_follower src/wall_follower.cpp)
add_executable(lap_time_server src/lap_time_server.cpp)
add_executable(lap_time_client src/lap_time_client.cpp)
ament_target_dependencies(
  wall_finder_server 
  rclcpp custom_interfaces
  sensor_msgs
  geometry_msgs
  rosidl_default_runtime
  rosidl_typesupport_cpp)

ament_target_dependencies(
  wall_follower 
  rclcpp custom_interfaces
  sensor_msgs
  geometry_msgs
  rosidl_default_runtime
  rosidl_typesupport_cpp)

ament_target_dependencies(
  lap_time_server 
  rclcpp 
  rclcpp_action
  nav_msgs
  custom_interfaces
  sensor_msgs
  geometry_msgs
  rosidl_default_runtime
  rosidl_typesupport_cpp)

ament_target_dependencies(
  lap_time_client 
  rclcpp 
  rclcpp_action
  nav_msgs
  custom_interfaces
  sensor_msgs
  geometry_msgs
  rosidl_default_runtime
  rosidl_typesupport_cpp)

    

include_directories(include
${rclcpp_INCLUDE_DIRS}
${custom_interfaces_INCLUDE_DIRS}
${geometry_msgs_INCLUDE_DIRS}
${sensor_msgs_INCLUDE_DIRS}
${rosidl_default_runtime_INCLUDE_DIRS}
${rosidl_typesupport_cpp_INCLUDE_DIRS})

install(TARGETS wall_finder_server wall_follower lap_time_server lap_time_client
DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)



ament_package()
