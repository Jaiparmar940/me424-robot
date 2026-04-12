"""
robot.launch.py
===============
Pi-side launch — starts the serial bridge and robot_state_publisher.
Run this on the Pi. Then run rviz.launch.py on your laptop.

Usage
-----
  ros2 launch ramses_robot_description robot.launch.py
  ros2 launch ramses_robot_description robot.launch.py port:=/dev/ttyUSB1
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:

    desc_pkg  = get_package_share_directory('ramses_robot_description')
    urdf_path = os.path.join(desc_pkg, 'urdf', 'robot.urdf')

    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    port_arg = DeclareLaunchArgument(
        'port', default_value='/dev/ttyUSB0',
        description='Serial port for the ESP32',
    )
    baudrate_arg = DeclareLaunchArgument(
        'baudrate', default_value='115200',
        description='Serial baud rate',
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
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
        robot_state_publisher,
        bridge_node,
    ])