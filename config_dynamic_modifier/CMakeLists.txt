cmake_minimum_required(VERSION 3.8)
project(config_dynamic_modifier)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets Core Gui)
find_package(rviz_common REQUIRED)
find_package(rviz_default_plugins REQUIRED)

include_directories(
        include
)

include_directories(include)
# 创建可执行文件
add_executable(config_dynamic_modifier_node src/config_dynamic_modifier_node.cpp)

# 链接依赖库
ament_target_dependencies(config_dynamic_modifier_node
  rclcpp
  geometry_msgs
  std_msgs
  nav2_util
)


# 安装可执行文件
install(TARGETS
  config_dynamic_modifier_node
  DESTINATION lib/${PROJECT_NAME}
)

# 安装launch文件
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# 安装配置文件
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

# 安装脚本文件
install(DIRECTORY
  scripts
  DESTINATION share/${PROJECT_NAME}
)


ament_package() 