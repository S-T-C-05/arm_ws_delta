"""Launch file: real hardware mode.

Starts the hardware control stack:
  - robot_state_publisher    (URDF + TF)
  - hardware_controller      (real RoboClaw, SparkFun, sensor reader)
  - stm32_serial_bridge      (STM32 firmware interface)

Usage:
    ros2 launch arm_controllers hardware.launch.py [rviz:=false]
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
    hw_config = os.path.join(arm_controllers_dir, 'config', 'hardware_controller.yaml')

    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str,
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'rviz',
            default_value='false',
            description='Launch RViz2 for monitoring',
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
            executable='hardware_controller',
            name='hardware_controller',
            output='screen',
            parameters=[hw_config],
        ),

        Node(
            package='arm_firmware',
            executable='stm32_serial_bridge',
            name='stm32_serial_bridge',
            output='screen',
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
    ])
