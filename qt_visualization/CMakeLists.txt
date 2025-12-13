cmake_minimum_required(VERSION 3.8)
project(qt_visualization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Qt5 configuration
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Qt5Core REQUIRED)
find_package(Qt5Widgets REQUIRED)
find_package(Qt5Gui REQUIRED)
find_package(Qt5Charts REQUIRED)
find_package(fake_capture_msgs REQUIRED)
find_package(event_bus_cpp REQUIRED)
find_package(data_processing REQUIRED)  # 添加对data_processing包的依赖

# Create library for VisualizationWindow
add_library(qt_visualization_lib SHARED
  src/visualization_window.cpp
  src/qt_sensor_data_node.cpp
  include/qt_visualization/visualization_window.hpp
  include/qt_visualization/qt_sensor_data_node.hpp
)

# Set target include directories
target_include_directories(qt_visualization_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Link libraries for the library target - 只保留Qt直接链接，event_bus_cpp由ament_target_dependencies处理
target_link_libraries(qt_visualization_lib
  Qt5::Core
  Qt5::Widgets
  Qt5::Gui
  Qt5::Charts
)

# Link ROS2 dependencies using ament_target_dependencies - 这里会自动处理event_bus_cpp的链接
ament_target_dependencies(qt_visualization_lib
  rclcpp
  rclcpp_components
  std_msgs
  fake_capture_msgs
  event_bus_cpp
  data_processing  # 添加对data_processing包的依赖
)

# Register the component
# Register only the SensorDataNode component, not VisualizationWindow
rclcpp_components_register_nodes(qt_visualization_lib 
  "qt_visualization::SensorDataNode")

# Add executable
add_executable(qt_visualization_node
  src/main.cpp
)

# Link the executable with the library
target_link_libraries(qt_visualization_node
  qt_visualization_lib
  Qt5::Core
  Qt5::Widgets
  pthread  # 添加线程支持
)

# Link ROS2 dependencies for executable
ament_target_dependencies(qt_visualization_node
  rclcpp
  data_processing  # 添加对data_processing包的依赖
)

# Install libraries
install(TARGETS qt_visualization_lib
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Install header files
install(DIRECTORY include/
  DESTINATION include
)

# Install executable
install(TARGETS qt_visualization_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Export targets
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp rclcpp_components std_msgs Qt5Core Qt5Widgets Qt5Gui Qt5Charts fake_capture_msgs event_bus_cpp data_processing)  # 添加对data_processing包的依赖

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