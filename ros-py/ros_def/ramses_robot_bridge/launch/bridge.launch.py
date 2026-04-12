"""
bridge.launch.py
================
Launch the ESP32 serial bridge node.

Usage
-----
  ros2 launch ramses_robot_bridge bridge.launch.py
  ros2 launch ramses_robot_bridge bridge.launch.py port:=/dev/ttyUSB1
  ros2 launch ramses_robot_bridge bridge.launch.py port:=/dev/ttyUSB0 baudrate:=115200
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='/dev/ttyUSB0',
        description='Serial port the ESP32 is connected to',
    )
    baudrate_arg = DeclareLaunchArgument(
        'baudrate',
        default_value='115200',
        description='Serial baud rate',
    )

    bridge_node = Node(
        package='ramses_robot_bridge',
        executable='ramses_robot_bridge',
        name='ramses_robot_bridge',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'port':     LaunchConfiguration('port'),
            'baudrate': LaunchConfiguration('baudrate'),
        }],
    )

    return LaunchDescription([
        port_arg,
        baudrate_arg,
        bridge_node,
    ])
