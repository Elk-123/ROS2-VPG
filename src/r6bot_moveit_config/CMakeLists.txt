cmake_minimum_required(VERSION 3.8)
project(r6bot_moveit_config)

find_package(ament_cmake REQUIRED)

# 安装 launch 和 config 文件夹到 share 目录
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
