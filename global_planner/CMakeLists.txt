cmake_minimum_required(VERSION 3.8)
project(global_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(standalone_global_planner src/standalone_global_planner.cpp)
target_include_directories(standalone_global_planner PRIVATE include)

ament_target_dependencies(
  standalone_global_planner
  geometry_msgs
  nav_msgs
  rclcpp
)

install(
  TARGETS standalone_global_planner
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
