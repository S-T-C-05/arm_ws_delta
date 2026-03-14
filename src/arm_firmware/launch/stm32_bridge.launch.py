"""Launch file for the STM32 serial bridge node.

Usage:
    ros2 launch arm_firmware stm32_bridge.launch.py [port:=/dev/ttyACM0]
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    arm_firmware_dir = get_package_share_directory('arm_firmware')
    bridge_config = os.path.join(arm_firmware_dir, 'config', 'stm32_bridge.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'port',
            default_value='/dev/ttyACM0',
            description='Serial port for the STM32 microcontroller',
        ),
        DeclareLaunchArgument(
            'baudrate',
            default_value='115200',
            description='Baud rate for the STM32 serial connection',
        ),

        Node(
            package='arm_firmware',
            executable='stm32_serial_bridge',
            name='stm32_serial_bridge',
            output='screen',
            parameters=[
                bridge_config,
                {
                    'port': LaunchConfiguration('port'),
                    'baudrate': LaunchConfiguration('baudrate'),
                },
            ],
        ),
    ])
