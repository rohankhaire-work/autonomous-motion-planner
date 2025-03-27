cmake_minimum_required(VERSION 3.15)
project(autonomous_motion_planner)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(rclcpp REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(nav_msgs REQUIRED)

add_executable(motion_planner src/motion_planner.cpp
                              src/planners/a_star.cpp)

target_include_directories(motion_planner PRIVATE include)
ament_target_dependencies(motion_planner rclcpp nav_msgs grid_map_ros)

ament_package()
