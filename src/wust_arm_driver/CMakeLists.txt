cmake_minimum_required(VERSION 3.10)
project(wust_arm_driver)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
add_definitions(-Wall -Wextra)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# 包含 include 目录
include_directories(include)

# 构建库
ament_auto_add_library(${PROJECT_NAME} SHARED
  src/wust_arm_driver.cpp
  src/crc.cpp
)

# 注册组件和生成节点可执行文件
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN wust_arm_driver::WustArmDriver
  EXECUTABLE ${PROJECT_NAME}_node
)

# 安装配置和启动文件
ament_auto_package(
  INSTALL_TO_SHARE
  config
  launch
)