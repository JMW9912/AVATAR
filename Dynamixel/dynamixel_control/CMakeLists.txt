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
# ── control_node ──────────────────────────────────────────────────────────
add_executable(read_write_node src/control_node/read_write_node.cpp)
ament_target_dependencies(read_write_node
  dynamixel_sdk rclcpp std_msgs sensor_msgs
)
add_executable(joint_state_publisher src/control_node/joint_state_publisher.cpp)
ament_target_dependencies(joint_state_publisher
  rclcpp sensor_msgs
)
add_executable(csv_joint_state_publisher src/control_node/csv_joint_state_publisher.cpp)
ament_target_dependencies(csv_joint_state_publisher
  rclcpp sensor_msgs std_msgs
)
add_executable(h1_csv_joint_state_publisher src/control_node/h1_csv_joint_state_publisher.cpp)
ament_target_dependencies(h1_csv_joint_state_publisher
  rclcpp sensor_msgs std_msgs
)
add_executable(torque_read_degree_write src/control_node/torque_read_degree_write.cpp)
ament_target_dependencies(torque_read_degree_write
  dynamixel_sdk rclcpp std_msgs sensor_msgs
)
# ── Poppy_robot ───────────────────────────────────────────────────────────
add_executable(poppy_read_write_node src/control_node/Poppy_robot/poppy_read_write_node.cpp)
ament_target_dependencies(poppy_read_write_node
  dynamixel_sdk rclcpp std_msgs sensor_msgs
)
add_executable(poppy_csv_joint_state_publisher src/control_node/Poppy_robot/poppy_csv_joint_state_publisher.cpp)
ament_target_dependencies(poppy_csv_joint_state_publisher
  rclcpp sensor_msgs std_msgs
)
# ── Avatar_robot ──────────────────────────────────────────────────────────
add_executable(avatar_read_write_node src/control_node/Avatar_robot/avatar_read_write_node.cpp)
ament_target_dependencies(avatar_read_write_node
  dynamixel_sdk rclcpp std_msgs sensor_msgs
)
add_executable(avatar_csv_joint_state_publisher src/control_node/Avatar_robot/avatar_csv_joint_state_publisher.cpp)
ament_target_dependencies(avatar_csv_joint_state_publisher
  rclcpp sensor_msgs std_msgs
)
# ── torque_control_node ───────────────────────────────────────────────────
add_executable(torque_compensation_node src/torque_control_node/torque_compensation_node.cpp)
ament_target_dependencies(torque_compensation_node
  dynamixel_sdk rclcpp std_msgs sensor_msgs
)
# ── visual_node ───────────────────────────────────────────────────────────
add_executable(rviz_visualization_node src/visual_node/rviz_visualization_node.cpp)
ament_target_dependencies(rviz_visualization_node
  rclcpp sensor_msgs geometry_msgs std_srvs urdf rviz_visual_tools tf2_eigen
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
  avatar_read_write_node
  avatar_csv_joint_state_publisher
  DESTINATION lib/${PROJECT_NAME}
)

# Python 스크립트 설치 (ros2 run dynamixel_control <script>.py 로 실행 가능)
install(PROGRAMS
  scripts/poppy_slider_gui.py
  scripts/poppy_fk_recorder.py
  scripts/avatar_slider_gui.py
  src/control_node/Poppy_robot/poppy_ros_relay_claude.py
  DESTINATION lib/${PROJECT_NAME}
)

# poppy_ros_relay_claude.py 가 sys.path 에 추가하는 공용 모듈 폴더
# (자기 파일 위치 기준 ./Robot_Retargeting 를 보므로 스크립트와 같은 곳에 설치해야 함)
install(DIRECTORY src/control_node/Poppy_robot/Robot_Retargeting
  DESTINATION lib/${PROJECT_NAME}/
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

# h1_2_description, poppy_description, avatar_description 은 urdf/ 하위로 이동됨
# install(DIRECTORY urdf ...) 가 이미 하위 폴더 전체를 커버함

# Test
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
