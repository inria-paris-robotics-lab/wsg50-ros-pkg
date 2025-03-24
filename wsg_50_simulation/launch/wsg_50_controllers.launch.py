#############################################################################################################
# Description: This file is used to launch the controllers for the WSG50 gripper.
# The controllers are loaded from the controller.yaml file and  for both the right and left fingers of the gripper.
# Arguments:
#   - prefix: Gripper prefix
#   - controller_file: Path to the WSG50 controller file
# Usage:
#   - ros2 launch wsg_50_driver wsg_50_controllers.launch.py prefix:=wsg_50 controller_file:=<path_to_controller_file>
#############################################################################################################
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

def launch_setup(context):
    # Arguments
    prefix = LaunchConfiguration('prefix').perform(context)
    controller_file=LaunchConfiguration('controller_file').perform(context)
    # Controller spawner
    right_finger_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[prefix+"wsg_50_gr", '--param-file', controller_file],
    )
    left_finger_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[prefix+"wsg_50_gl", '--param-file', controller_file],
    )
    return [right_finger_controller_spawner, left_finger_controller_spawner]

def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value="",
            description="Gripper prefix"
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "controller_file",
            default_value="",
            description="Path to the WSG50 controller file"
        )
    )
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])


