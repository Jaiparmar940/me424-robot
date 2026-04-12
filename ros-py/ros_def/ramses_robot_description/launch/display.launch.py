"""
display.launch.py
=================
Launches the full visualisation stack:
  1. robot_state_publisher  — reads URDF, publishes TF transforms
  2. ramses_robot_bridge     — serial bridge, publishes /arm/joint_states
  3. rviz2                   — visualises the arm live

Usage
-----
  ros2 launch ramses_robot_description display.launch.py
  ros2 launch ramses_robot_description display.launch.py port:=/dev/ttyUSB1
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:

    # --- Paths ---
    desc_pkg  = get_package_share_directory('ramses_robot_description')
    urdf_path = os.path.join(desc_pkg, 'urdf', 'robot.urdf')

    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    # --- Args ---
    port_arg = DeclareLaunchArgument(
        'port', default_value='/dev/ttyUSB0',
        description='Serial port for the ESP32',
    )
    baudrate_arg = DeclareLaunchArgument(
        'baudrate', default_value='115200',
        description='Serial baud rate',
    )

    # --- Nodes ---

    # Reads URDF and publishes TF from /arm/joint_states
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )

    # Publishes zero for any joints not yet covered by the bridge.
    # This prevents "No Transform" errors in RViz2 on startup.
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'source_list': ['/joint_states'],
        }],
    )

    # Serial bridge — publishes /joint_states
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

    # RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
    )

    return LaunchDescription([
        port_arg,
        baudrate_arg,
        robot_state_publisher,
        bridge_node,
        rviz_node,
    ])