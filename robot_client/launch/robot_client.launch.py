from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("birc_config", package_name="arm2").to_moveit_configs()

    # MoveGroupInterface demo executable
    move_group_demo = Node(
        #name="birc_move_group_interface_tutorial",
        name="move_ee",
        package="robot_client",
        #executable="birc_move_group_interface_tutorial",
        executable="move_ee",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
    )

    return LaunchDescription([move_group_demo])
