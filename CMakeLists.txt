cmake_minimum_required(VERSION 3.8)
project(var_kqt_beadando)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Add speed_sensor_node executable
add_executable(speed_sensor_node src/speed_sensor_node.cpp)
ament_target_dependencies(speed_sensor_node rclcpp std_msgs)

# Add cruise_control_node executable
add_executable(cruise_control_node src/cruise_control_node.cpp)
ament_target_dependencies(cruise_control_node rclcpp std_msgs)

# Install targets
install(TARGETS
  speed_sensor_node
  cruise_control_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()