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
from launch_ros.parameter_descriptions import ParameterFile

def generate_launch_description():
    
    urdf_file = os.path.join(get_package_share_directory('wsg_50_simulation'),'urdf','wsg50.urdf.xacro')
    rviz_config_file = os.path.join(get_package_share_directory('wsg_50_simulation'),'rviz','wsg50.rviz')
    controller_file=os.path.join(get_package_share_directory('wsg_50_driver'),'config','controller.yaml')

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            urdf_file,
            " "
            "real_hardware:=",
            "true",
            " ",
            "sim_standalone:=",
            "true",
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
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='both',
        parameters=[ParameterFile(controller_file)],
    )
    # driver = Node(
    #     package='wsg_50_driver',
    #     executable='wsg_50_driver',
    #     output='both',
    # )
    # RVIZ

    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=["joint_state_broadcaster"],
    )

    gripper_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=["gripper_controller"],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        output='screen',
        arguments=["-d", rviz_config_file],
    )
    return LaunchDescription([
        node_robot_state_publisher,
        # driver,
        rviz_node,
        ros2_control_node,
        joint_state_broadcaster,
        gripper_controller,
    ])


