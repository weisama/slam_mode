cmake_minimum_required(VERSION 3.8)
project(visualize_delay)

# 检查编译器
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 寻找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# 设置包含目录
include_directories(include)

# 创建可执行文件
add_executable(visualize_delay_node src/visualize_delay_node.cpp)

# 指定目标链接库
target_link_libraries(visualize_delay_node
  ${rclcpp_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
  ${geometry_msgs_LIBRARIES}
  ${tf2_LIBRARIES}
  ${visualization_msgs_LIBRARIES}
  ${std_msgs_LIBRARIES}
)

# 添加依赖
ament_target_dependencies(visualize_delay_node
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs
  visualization_msgs
  std_msgs
)

# 安装目标
install(TARGETS
  visualize_delay_node
  DESTINATION lib/${PROJECT_NAME}
)


# 安装配置文件
install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

# 导出依赖
ament_export_dependencies(
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs
  visualization_msgs
  std_msgs
)

# 导出包含目录
ament_export_include_directories(include)

# 构建包
ament_package()
