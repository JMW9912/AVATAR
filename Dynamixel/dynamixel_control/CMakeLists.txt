cmake_minimum_required(VERSION 3.8)
project(dynamixel_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Set C++ Standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(dynamixel_sdk REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
# URDF
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(rviz_visual_tools REQUIRED)
find_package(tf2_eigen REQUIRED)
# find_package(moveit_ros_planning_interface REQUIRED)

include_directories(include
  ${std_msgs_INCLUDE_DIRS}
)

# Build
add_executable(read_write_node src/read_write_node.cpp)
ament_target_dependencies(read_write_node
  dynamixel_sdk
  rclcpp
  std_msgs
  sensor_msgs
  # moveit_ros_planning_interface
)
add_executable(poppy_read_write_node src/poppy_read_write_node.cpp)
ament_target_dependencies(poppy_read_write_node
  dynamixel_sdk
  rclcpp
  std_msgs
  sensor_msgs
  # moveit_ros_planning_interface
)
add_executable(joint_state_publisher src/joint_state_publisher.cpp)
ament_target_dependencies(joint_state_publisher
  rclcpp
  sensor_msgs
)
add_executable(csv_joint_state_publisher src/csv_joint_state_publisher.cpp)
ament_target_dependencies(csv_joint_state_publisher
  rclcpp
  sensor_msgs
  std_msgs
)
add_executable(h1_csv_joint_state_publisher src/h1_csv_joint_state_publisher.cpp)
ament_target_dependencies(h1_csv_joint_state_publisher
  rclcpp
  sensor_msgs
  std_msgs
)
add_executable(poppy_csv_joint_state_publisher src/poppy_csv_joint_state_publisher.cpp)
ament_target_dependencies(poppy_csv_joint_state_publisher
  rclcpp
  sensor_msgs
  std_msgs
)
# RViz 시각화 노드 추가
add_executable(rviz_visualization_node src/rviz_visualization_node.cpp)
ament_target_dependencies(rviz_visualization_node
  rclcpp
  sensor_msgs
  geometry_msgs
  std_srvs
  urdf
  rviz_visual_tools
  tf2_eigen
)
add_executable(torque_compensation_node src/torque_compensation_node.cpp)
ament_target_dependencies(torque_compensation_node
  dynamixel_sdk
  rclcpp
  std_msgs
  sensor_msgs
)
add_executable(torque_read_degree_write src/torque_read_degree_write.cpp)
ament_target_dependencies(torque_read_degree_write
  dynamixel_sdk
  rclcpp
  std_msgs
  sensor_msgs
)

# Install
install(TARGETS
  read_write_node
  poppy_read_write_node
  joint_state_publisher
  csv_joint_state_publisher
  h1_csv_joint_state_publisher
  poppy_csv_joint_state_publisher
  #DIRECTORY data
  rviz_visualization_node
  torque_compensation_node
  torque_read_degree_write
  DESTINATION lib/${PROJECT_NAME}
)

# Install 런치 파일
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install 데이터 파일
install(DIRECTORY data
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY h1_2_description
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY poppy_description
  DESTINATION share/${PROJECT_NAME}/
)

# Test
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
