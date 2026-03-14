"""Launch file: simulation mode.

Starts the full simulation stack:
  - robot_state_publisher   (URDF + TF)
  - mock_hardware_controller (simulated joint feedback, no hardware needed)
  - arm_keyboard_teleop     (optional teleop)
  - RViz2

Usage:
    ros2 launch arm_controllers sim.launch.py [rviz:=true]
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    arm_core_dir = get_package_share_directory('arm_core')
    arm_controllers_dir = get_package_share_directory('arm_controllers')

    xacro_file = os.path.join(arm_core_dir, 'urdf', 'arm.xacro')
    mock_config = os.path.join(arm_controllers_dir, 'config', 'mock_controller.yaml')

    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str,
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='Launch RViz2',
        ),
        DeclareLaunchArgument(
            'teleop',
            default_value='false',
            description='Launch keyboard teleop node',
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),

        Node(
            package='arm_controllers',
            executable='mock_controller',
            name='mock_hardware_controller',
            output='screen',
            parameters=[mock_config],
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),

        Node(
            package='arm_core',
            executable='arm_keyboard_teleop.py',
            name='arm_keyboard_teleop',
            output='screen',
            remappings=[('joint_states', 'arm_joint_commands')],
            condition=IfCondition(LaunchConfiguration('teleop')),
        ),
    ])
