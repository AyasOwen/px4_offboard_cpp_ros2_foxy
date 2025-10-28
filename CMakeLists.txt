cmake_minimum_required(VERSION 3.5)
project(offboard_cpp)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)

# 包含头文件目录
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

# ---------------------------------------------
# 构建基础类 OffboardControlNode 为库
add_library(offboard_control_node_lib
  src/offboard_control_node.cpp
)

target_include_directories(offboard_control_node_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

ament_target_dependencies(offboard_control_node_lib
  rclcpp
  std_msgs
  geometry_msgs
  mavros_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

target_link_libraries(offboard_control_node_lib
  ${OpenCV_LIBS}
)

# ---------------------------------------------
# 主测试节点：offboard_node
add_executable(offboard_node
  src/offboard_demo.cpp
)

target_link_libraries(offboard_node
  offboard_control_node_lib
)

ament_target_dependencies(offboard_node
  rclcpp
  std_msgs
)

# ---------------------------------------------
# 子节点 1：2024_Ti_demo_node
add_executable(2024_Ti_demo_node
  src/2024_Ti_demo.cpp
)

target_link_libraries(2024_Ti_demo_node
  offboard_control_node_lib
)

ament_target_dependencies(2024_Ti_demo_node
  rclcpp
  std_msgs
)

# ---------------------------------------------
# 子节点 2：2025_Ti_main_node
add_executable(2025_Ti_main_node
  src/2025_Ti_main.cpp
)

target_link_libraries(2025_Ti_main_node
  offboard_control_node_lib
)

ament_target_dependencies(2025_Ti_main_node
  rclcpp
  std_msgs
)

# ---------------------------------------------
# 安装所有可执行文件
install(TARGETS
  offboard_node
  2024_Ti_demo_node
  2025_Ti_main_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
