from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterFile
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare




def launch_setup(context):
    # Initialize Arguments
    prefix = LaunchConfiguration('prefix').perform(context)
    
    wsg50_controller_file=PathJoinSubstitution([FindPackageShare('wsg_50_simulation'),'controllers', 'wsg50_integrate.yaml'])

    right_finger_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[prefix+"wsg_50_gr", '--param-file', wsg50_controller_file],
    )

    left_finger_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[prefix+"wsg_50_gl", '--param-file', wsg50_controller_file],
    )

    return [right_finger_controller_spawner, left_finger_controller_spawner]


def generate_launch_description():
    declared_arguments = []
    # UR specific arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value="",
            description="Gripper prefix"
        )
    )
    
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])


