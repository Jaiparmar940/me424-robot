"""
calibrate.py
============
Calibration helper — run this after homing the arm to determine
the JOINT_ZERO_OFFSETS values for arm_model.py.

Usage
-----
  # 1. Home all axes first:
  ros2 topic pub --once /arm/command std_msgs/msg/String "data: 'home s2'"
  ros2 topic pub --once /arm/command std_msgs/msg/String "data: 'home s3'"
  # etc.

  # 2. Run calibration:
  ros2 run ramses_ik calibrate
"""

import os
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from .arm_model import ArmModel, print_calibration_report, JOINT_ZERO_OFFSETS
import math


def main(args=None) -> None:
    rclpy.init(args=args)

    pkg = get_package_share_directory('ramses_robot_description')
    urdf_path = os.path.join(pkg, 'urdf', 'robot.urdf')

    print(f'Loading URDF from: {urdf_path}')
    arm = ArmModel(urdf_path)
    print_calibration_report(arm)

    print('\nFK at physical zeros (with current offsets applied):')
    zeros_physical = {j: 0.0 for j in ['S1', 'S2', 'S3', 'S4', 'S5']}
    # Apply offsets to get URDF angles at physical zero
    from .arm_model import physical_to_urdf
    zeros_urdf = {j: physical_to_urdf(j, 0.0) for j in zeros_physical}
    fk = arm.forward_kinematics(zeros_urdf)
    x, y, z = fk[0, 3], fk[1, 3], fk[2, 3]
    print(f'  End effector at physical zero: x={x:.4f}  y={y:.4f}  z={z:.4f} m')
    print(f'  ({x/0.0254:.3f}", {y/0.0254:.3f}", {z/0.0254:.3f}")')

    print('\nExpected physical zero position (from robot_dimensions.txt):')
    print('  S1=0: arm pointing towards +X (docking station direction)')
    print('  S2=S3=S4=0: arm pointing straight up')
    print('  S5=0: default wrist orientation')
    total_z = (6.335 + 10.0 + 8.0 + 3.209) * 0.0254
    print(f'  Expected end effector roughly above origin at z ≈ {total_z:.3f} m')

    rclpy.shutdown()


if __name__ == '__main__':
    main()