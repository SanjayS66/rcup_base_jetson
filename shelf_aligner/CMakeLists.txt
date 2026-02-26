cmake_minimum_required(VERSION 3.8)
project(shelf_aligner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(laser_line_extraction_ros2 REQUIRED)

add_executable(simple_aligner src/allign_to_shelf.cpp)
ament_target_dependencies(simple_aligner 
  rclcpp 
  geometry_msgs 
  tf2_ros 
  tf2_geometry_msgs 
  laser_line_extraction_ros2
)

install(TARGETS simple_aligner
  DESTINATION lib/${PROJECT_NAME})

ament_package()
