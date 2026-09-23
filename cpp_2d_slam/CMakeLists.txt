cmake_minimum_required(VERSION 3.8)
project(cpp_2d_slam)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(
  include
  include/${PROJECT_NAME}
  include/slam
  include/slam/base
)

# find ROS dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# find Qt dependencies
find_package(Qt6 REQUIRED COMPONENTS Gui Core Widgets)

find_path(G2O_INCLUDE_DIR NAMES g2o/core/sparse_optimizer.h)
find_library(G2O_CORE_LIBRARY NAMES g2o_core)
find_library(G2O_STUFF_LIBRARY NAMES g2o_stuff)
find_library(G2O_TYPES_SLAM2D_LIBRARY NAMES g2o_types_slam2d)
find_library(G2O_SOLVER_EIGEN_LIBRARY NAMES g2o_solver_eigen)

set(ROS_DEPEND
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
)

set(QtLib
  Qt6::Core
  Qt6::Gui
  Qt6::Widgets
  Eigen3::Eigen
)

set(SLAM_CORE_SOURCES
  src/painter.cpp
  src/bridge.cpp
  src/${PROJECT_NAME}/teleopt.cpp
  src/slam/base/scan_match.cpp
  src/slam/base/slam_basic.cpp
  src/slam/base/my_pose_graph.cpp
  src/slam/base/map_backend.cpp
  src/slam/scan_match_backend.cpp
  src/slam/loop_detecter.cpp
  src/slam/slam.cpp
  include/painter.h
  include/bridge.h
  include/slam/scan_match_backend.h
  include/slam/loop_detecter.h
  include/slam/slam.h
)

set(APP_SOURCES
  src/main.cpp
  src/${PROJECT_NAME}/imu_receive_node.cpp
  src/${PROJECT_NAME}/sensor_receive_node.cpp
  src/${PROJECT_NAME}/odom_receive_node.cpp
  src/${PROJECT_NAME}/plan_receive_node.cpp
)

add_library(${PROJECT_NAME}_core STATIC
  ${SLAM_CORE_SOURCES}
)

ament_target_dependencies(${PROJECT_NAME}_core
  ${ROS_DEPEND}
)

target_link_libraries(${PROJECT_NAME}_core ${QtLib})

add_executable(${PROJECT_NAME}
  ${APP_SOURCES}
)

set_target_properties(${PROJECT_NAME} PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
)

ament_target_dependencies(${PROJECT_NAME}
  ${ROS_DEPEND}
)

target_link_libraries(${PROJECT_NAME} ${PROJECT_NAME}_core)

if(G2O_INCLUDE_DIR AND G2O_CORE_LIBRARY AND G2O_STUFF_LIBRARY AND G2O_TYPES_SLAM2D_LIBRARY AND G2O_SOLVER_EIGEN_LIBRARY)
  target_include_directories(${PROJECT_NAME}_core PRIVATE ${G2O_INCLUDE_DIR})
  target_compile_definitions(${PROJECT_NAME}_core PRIVATE CPP_2D_SLAM_HAS_G2O=1)
  target_link_libraries(${PROJECT_NAME}_core
    ${G2O_CORE_LIBRARY}
    ${G2O_STUFF_LIBRARY}
    ${G2O_TYPES_SLAM2D_LIBRARY}
    ${G2O_SOLVER_EIGEN_LIBRARY}
  )
  message(STATUS "g2o found: enabling g2o pose graph optimization with Huber kernel on loop edges")
else()
  message(STATUS "g2o not found: using built-in pose graph optimizer")
endif()

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_cpp_2d_slam
    test/test_pose_graph.cpp
    test/test_slam_threads.cpp
  )
  target_link_libraries(test_cpp_2d_slam ${PROJECT_NAME}_core)
  set_tests_properties(test_cpp_2d_slam PROPERTIES
    ENVIRONMENT "QT_QPA_PLATFORM=offscreen"
  )
endif()

install(
  TARGETS
    ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
