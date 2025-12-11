cmake_minimum_required(VERSION 3.8)
project(data_processing)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Qt5Core REQUIRED)

# Qt related configuration
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# Include directories
# 移除全局include_directories
# include_directories(include)

# 其他配置保持不变

# Create library for SoftwareFIFO and DataProcessingNode
add_library(data_processing_lib
  src/software_fifo.cpp
  src/data_processing_node.cpp
  include/data_processing/software_fifo.hpp
  include/data_processing/data_processing_node.hpp
)

# Set target include directories
target_include_directories(data_processing_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Link libraries for the library target
target_link_libraries(data_processing_lib
  Qt5::Core
)

# Link ROS2 dependencies using ament_target_dependencies
ament_target_dependencies(data_processing_lib
  rclcpp
  std_msgs
)

# Add executable - main.cpp doesn't need MOC
add_executable(data_processing_node 
  src/main.cpp
)

# Link the executable with the library and Qt
target_link_libraries(data_processing_node
  data_processing_lib
  Qt5::Core
  pthread  # Needed for std::thread
)

# Link ROS2 dependencies for executable
ament_target_dependencies(data_processing_node
  rclcpp
)

# Install libraries
install(TARGETS data_processing_lib
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
install(TARGETS data_processing_node
  DESTINATION lib/${PROJECT_NAME}
)

# Export targets
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp std_msgs Qt5Core)

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