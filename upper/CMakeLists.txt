cmake_minimum_required(VERSION 3.5)
project(upper)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# -----------------------------------------------------------------------------
# 查找依赖
# -----------------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(yaml-cpp REQUIRED)

# -----------------------------------------------------------------------------
# 编译节点
# -----------------------------------------------------------------------------
add_executable(${PROJECT_NAME}_node src/upper.cpp)

ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  nav_msgs
  sensor_msgs
  tf2_msgs
  std_msgs
)

# 链接 JSON 和 YAML 库
target_link_libraries(${PROJECT_NAME}_node
  yaml-cpp
  nlohmann_json::nlohmann_json
)

# -----------------------------------------------------------------------------
# 安装
# -----------------------------------------------------------------------------
install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

