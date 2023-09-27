cmake_minimum_required(VERSION 3.5)
project(atl_msgs)

set(CMAKE_CXX_STANDARD 17)
add_compile_options(-Werror -Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 3.3 REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ServoInput.msg"
  "msg/ServosInput.msg"
  "msg/ServoFeedback.msg"
  "msg/ServosFeedback.msg"
  "msg/Depth.msg"
  "msg/Leak.msg"
  
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  builtin_interfaces
)

ament_export_dependencies(
  rosidl_default_runtime
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  visualization_msgs
  std_srvs
  eigen3_cmake_module
  Eigen3
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_package()
