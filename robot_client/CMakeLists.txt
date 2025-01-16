cmake_minimum_required(VERSION 3.8)
project(robot_client)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_gripper REQUIRED)
#find_package(gripper2 REQUIRED)
find_package(geometry_msgs REQUIRED)
#find_package(ros2_aruco_interfaces REQUIRED)
#find_package(trac_ik_kinematics_plugin REQUIRED)
#find_package(pluginlib REQUIRED)
#find_package(kdl_parser REQUIRED)


add_executable(move_ee src/move_ee.cpp)
add_executable(birc_move_group_interface_tutorial src/birc_move_group_interface_tutorial.cpp)
add_executable(publisher src/publisher.cpp)
add_executable(moveit_control src/moveit_control.cpp)
#add_executable(kdl_solver src/kdl_solver.cpp)

#ament_target_dependencies(
#  kdl_solver
#  "rclcpp"
#  "moveit_core"
#  "moveit_ros_planning"
#  "moveit_ros_planning_interface"
#  "pluginlib"
#  "kdl_parser"
#)

target_compile_features(moveit_control PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  moveit_control
  "moveit_ros_planning_interface"
  "moveit_msgs"
  "rclcpp"
  "moveit_gripper"
  "geometry_msgs"
)

target_include_directories(move_ee PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(move_ee PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  move_ee
  "moveit_ros_planning_interface"
  "rclcpp"
  "moveit_gripper"
  "geometry_msgs"
)


ament_target_dependencies(publisher "rclcpp" "geometry_msgs")
target_include_directories(birc_move_group_interface_tutorial PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
  
target_compile_features(birc_move_group_interface_tutorial PUBLIC c_std_99 cxx_std_17)  
ament_target_dependencies(
  birc_move_group_interface_tutorial
  "moveit_ros_planning_interface"
  "moveit_msgs"
  "rclcpp"
  "moveit_gripper"
  "geometry_msgs"
  #"trac_ik_kinematics_plugin"
)

#install(TARGETS kdl_solver DESTINATION lib/${PROJECT_NAME})
  
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(TARGETS moveit_control
  DESTINATION lib/${PROJECT_NAME})
  
install(TARGETS move_ee
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS birc_move_group_interface_tutorial
  DESTINATION lib/${PROJECT_NAME})
  
install(TARGETS publisher
  DESTINATION lib/${PROJECT_NAME})

  
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
