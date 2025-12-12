# 删除第一行注释
cmake_minimum_required(VERSION 3.8)
project(event_bus_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 设置C++标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Qt5 COMPONENTS Core Widgets REQUIRED)

# Qt 自动处理 - 添加这4行配置
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# 添加库
add_library(event_bus_cpp SHARED
  src/event_bus.cpp
  include/event_bus_cpp/event_bus.hpp
)

# 设置包含目录
target_include_directories(event_bus_cpp PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# 链接依赖 - 使用ament_target_dependencies确保依赖传递
ament_target_dependencies(event_bus_cpp rclcpp)
target_link_libraries(event_bus_cpp
  Qt5::Core
  Qt5::Widgets
)

# 安装库
install(TARGETS event_bus_cpp
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# 安装头文件
install(DIRECTORY include/
  DESTINATION include
)

# 导出目标和依赖 - 添加include目录导出
ament_export_include_directories(include)
ament_export_libraries(event_bus_cpp)
ament_export_dependencies(rclcpp Qt5Core Qt5Widgets)

ament_package()