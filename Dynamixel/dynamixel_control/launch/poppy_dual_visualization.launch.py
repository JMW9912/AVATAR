"""
poppy_dual_visualization.launch.py

반투명 목표 모델 + 불투명 실제 모델 이중 시각화 런치 파일.

TF 구조:
  world ──(static)──▶ target/pelvis ──▶ target/chest ──▶ ...   (목표 각도)
  world ──(static)──▶ actual/pelvis ──▶ actual/chest ──▶ ...   (실제 측정값)

실행 방법:
  터미널 1: ros2 launch dynamixel_control poppy_dual_visualization.launch.py
  터미널 2: ros2 run dynamixel_control poppy_csv_joint_state_publisher
            (→ /joint_states 에 목표 각도 발행)
  터미널 3: ros2 run dynamixel_control poppy_read_write_node
            (→ /joint_states_actual 에 실측 각도 발행, 하드웨어 필요)
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from pathlib import Path


def generate_launch_description():
    pkg = get_package_share_directory("dynamixel_control")
    urdf_path = os.path.join(pkg, "poppy_description/robots", "Poppy_Humanoid.URDF")
    rviz_config_path = os.path.join(pkg, "urdf", "poppy_dual_rviz_config.rviz")
    robot_description = Path(urdf_path).read_text()

    return LaunchDescription([
        # ── 목표 각도 robot_state_publisher (/joint_states → TF prefix: target/) ──
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            namespace="target",
            name="robot_state_publisher",
            output="screen",
            parameters=[{
                "robot_description": robot_description,
                "frame_prefix": "target/",
            }],
            remappings=[("joint_states", "/joint_states")],
        ),

        # ── 실제 각도 robot_state_publisher (/joint_states_actual → TF prefix: actual/) ──
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            namespace="actual",
            name="robot_state_publisher",
            output="screen",
            parameters=[{
                "robot_description": robot_description,
                "frame_prefix": "actual/",
            }],
            remappings=[("joint_states", "/joint_states_actual")],
        ),

        # ── world → target/pelvis (두 로봇 모두 world 기준으로 고정) ──
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="world_to_target_pelvis",
            arguments=["0", "0", "0", "0", "0", "0", "world", "target/pelvis"],
        ),

        # ── world → actual/pelvis ──
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="world_to_actual_pelvis",
            arguments=["0", "0", "0", "0", "0", "0", "world", "actual/pelvis"],
        ),

        # ── RViz2 ──
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", rviz_config_path],
        ),
    ])
