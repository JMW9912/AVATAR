from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from pathlib import Path

def generate_launch_description():
    pkg = get_package_share_directory('dynamixel_control')
    urdf_file_path  = os.path.join(pkg, 'poppy_description/robots', 'Poppy_Humanoid.URDF')
    rviz_config_path = os.path.join(pkg, 'urdf', 'rviz_config.rviz')

    robot_description = Path(urdf_file_path).read_text()

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path]
        ),
    ])
