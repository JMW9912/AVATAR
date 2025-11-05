from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    package_share_directory = get_package_share_directory('dynamixel_control')
    # URDF 및 RViz 설정 파일 경로
    
    urdf_file_path = os.path.join(package_share_directory, 'poppy_description/robots', 'Poppy_Humanoid.URDF')
    rviz_config_path = os.path.join(package_share_directory, 'urdf', 'rviz_config.rviz')
    
    print(f"{package_share_directory}, /// , {urdf_file_path}")

    return LaunchDescription([
        # robot_state_publisher 노드
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': open(urdf_file_path).read()}]
        ),

        # # csv_joint_state_publisher 노드
        # Node(
        #     package='dynamixel_control',
        #     executable='csv_joint_state_publisher',
        #     name='csv_joint_state_publisher',
        #     output='screen'
        # ),

        # # RViz 시각화 노드
        # Node(
        #     package='dynamixel_control',
        #     executable='rviz_visualization_node',
        #     name='rviz_visualization_node',
        #     output='screen'
        # ),

        # RViz 실행
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path]
        ),
    ])
