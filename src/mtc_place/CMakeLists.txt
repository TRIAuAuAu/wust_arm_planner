cmake_minimum_required(VERSION 3.8)
project(mtc_place)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(moveit_task_constructor_msgs REQUIRED)
find_package(moveit_visual_tools REQUIRED)

add_executable(mtc_place_node src/mtc_place_node.cpp)

ament_target_dependencies(mtc_place_node
  rclcpp
  moveit_core
  moveit_ros_planning_interface
  moveit_task_constructor_core
  moveit_msgs
  tf2_geometry_msgs
  tf2_eigen
  Eigen3
)


install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)
install(TARGETS mtc_place_node
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
