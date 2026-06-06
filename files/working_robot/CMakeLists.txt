cmake_minimum_required(VERSION 3.8)
project(working_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)

# C++ executable
add_executable(joint_serial_bridge src/joint_serial_bridge.cpp)
ament_target_dependencies(joint_serial_bridge rclcpp sensor_msgs moveit_msgs)

# Install executable
install(
  TARGETS joint_serial_bridge
  DESTINATION lib/${PROJECT_NAME}
)

# Install folders
install(
  DIRECTORY launch urdf meshes config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
