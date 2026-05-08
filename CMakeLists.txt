cmake_minimum_required(VERSION 3.8)
project(robot_nodes)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# ── C++ node: scan_publisher ──────────────────────────────
add_executable(scan_publisher src/scan_publisher.cpp)
ament_target_dependencies(scan_publisher
  rclcpp sensor_msgs std_msgs)
target_include_directories(scan_publisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

install(TARGETS scan_publisher
  DESTINATION lib/${PROJECT_NAME})

# ── Python nodes ──────────────────────────────────────────
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  robot_nodes/vision_node.py
  robot_nodes/navigation_node.py
  robot_nodes/odometry_node.py
  DESTINATION lib/${PROJECT_NAME})

# ── Config & launch ───────────────────────────────────────
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY config launch maps
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY config launch maps urdf worlds
  DESTINATION share/${PROJECT_NAME})


ament_package()
