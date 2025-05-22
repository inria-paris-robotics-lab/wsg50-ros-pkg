############################################################################################################
# Description: This file is used to vizualize the WSG50 gripper in RVIZ.
# Usage:
# ros2 launch wsg_50_simulation view_wsg50.launch.py
############################################################################################################
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
import os

def generate_launch_description():
    
    urdf_file = os.path.join(get_package_share_directory('wsg_50_simulation'),'urdf','wsg50.urdf.xacro')
    rviz_config_file = os.path.join(get_package_share_directory('wsg_50_simulation'),'rviz','wsg50.rviz')

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            urdf_file,
            " ",
            "real_hardware:=",
            "false ",
            " ",
            "sim_standalone:=",
            "true ",
            " ",
        ]
    )

    robot_description = {"robot_description": robot_description_content}

    # Robot state publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description],
    )
    # Joint state publisher
    joint_state_publisher = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher',
        output='screen',
    )
    # RVIZ
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        output='screen',
        arguments=["-d", rviz_config_file],
    )
    return LaunchDescription([
        node_robot_state_publisher,
        joint_state_publisher,
        rviz_node,
    ])
