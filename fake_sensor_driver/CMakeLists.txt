cmake_minimum_required(VERSION 3.8)
project(fake_sensor_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(fake_capture_msgs REQUIRED)

# 创建库
add_library(fake_sensor_node_lib
  src/fake_sensor_node.cpp
)

# 使用目标包含目录（推荐做法）
target_include_directories(fake_sensor_node_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# 设置库的依赖
ament_target_dependencies(fake_sensor_node_lib rclcpp std_msgs fake_capture_msgs)

# 创建可执行文件
add_executable(fake_sensor_node 
  src/main.cpp
)

# 链接库到可执行文件
target_link_libraries(fake_sensor_node fake_sensor_node_lib)

# 设置可执行文件的依赖
ament_target_dependencies(fake_sensor_node rclcpp std_msgs)

# 安装库
install(TARGETS
  fake_sensor_node_lib
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# 安装可执行文件
install(TARGETS
  fake_sensor_node
  DESTINATION lib/${PROJECT_NAME})

# 安装头文件
install(DIRECTORY include/
  DESTINATION include)

# 导出库配置
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp std_msgs)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()