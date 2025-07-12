cmake_minimum_required(VERSION 3.5)
project(offboard_cpp)

# 默认使用 C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
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

# 添加可执行文件
add_executable(
  offboard_node src/offboard_control_node.cpp
  )

# 链接依赖库
ament_target_dependencies(offboard_node
  rclcpp
  geometry_msgs
  mavros_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

target_link_libraries(offboard_node
  ${OpenCV_LIBS}
)

# 安装节点
install(TARGETS
  offboard_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
