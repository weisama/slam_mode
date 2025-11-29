cmake_minimum_required(VERSION 3.5)
project(px4_launch)

find_package(ament_cmake REQUIRED)

# 安装 launch/ 到 install/share/px4_launch/launch/
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
