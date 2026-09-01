cmake_minimum_required(VERSION 3.8)
project(local_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(
  standalone_local_controller
  src/local_costmap.cpp
  src/standalone_local_controller.cpp
)
target_include_directories(standalone_local_controller PRIVATE include)

ament_target_dependencies(
  standalone_local_controller
  geometry_msgs
  nav_msgs
  rclcpp
  sensor_msgs
)

install(
  TARGETS standalone_local_controller
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

ament_package()
