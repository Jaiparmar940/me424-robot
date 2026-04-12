"""
ik_solver.py
============
ROS2 node that solves IK and sends motion commands to the arm bridge.

Subscribed topics
-----------------
/arm/target_xyztheta  (std_msgs/Float64MultiArray)
    Simplified [x, y, z, theta] target — end effector pointing straight
    down, theta = S5 wrist twist in radians. Most common use case.

/arm/target_pose  (geometry_msgs/Pose)
    Full 6-DOF target pose for arbitrary orientations.

/joint_states  (sensor_msgs/JointState)
    Current joint positions (from bridge). Used as IK initial guess.

Published topics
----------------
/arm/ik_status  (std_msgs/String)
    Status: SOLVING | SENDING | DONE | UNREACHABLE | ERR

Parameters
----------
urdf_path  (string)  Path to robot URDF. Defaults to installed
                     ramses_robot_description URDF.
"""

import math
import threading
from typing import Optional

import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String

from ramses_robot_interfaces.action import ArmMove

from .arm_model import ArmModel, angles_to_syncabs, firmware_pos_to_urdf_angles


class IKSolverNode(Node):

    # URDF joint name → stage name
    _JOINT_MAP = {
        's1_turntable':   'S1',
        's2_shoulder':    'S2',
        's3_elbow':       'S3',
        's4_wrist':       'S4',
        's5_wrist_twist': 'S5',
    }

    def __init__(self) -> None:
        super().__init__('ik_solver')

        # --- Parameters ---
        self.declare_parameter('urdf_path', '')
        urdf_path = self.get_parameter('urdf_path').get_parameter_value().string_value

        if not urdf_path:
            try:
                from ament_index_python.packages import get_package_share_directory
                import os
                pkg = get_package_share_directory('ramses_robot_description')
                urdf_path = os.path.join(pkg, 'urdf', 'robot.urdf')
            except Exception as e:
                self.get_logger().error(f'Could not find URDF: {e}')
                raise

        self.get_logger().info(f'Loading arm model from: {urdf_path}')
        self._arm = ArmModel(urdf_path)
        self.get_logger().info('Arm model loaded.')

        # --- State ---
        self._current_angles: dict = {j: 0.0 for j in ['S1','S2','S3','S4','S5']}
        self._state_lock = threading.Lock()
        self._busy = False

        # --- Callback group ---
        self._cbg = ReentrantCallbackGroup()

        # --- Action client ---
        self._action_client = ActionClient(
            self, ArmMove, '/arm/move',
            callback_group=self._cbg,
        )

        # --- Publishers ---
        self._pub_status = self.create_publisher(
            String, '/arm/ik_status', 10,
            callback_group=self._cbg,
        )

        # --- Subscribers ---
        self.create_subscription(
            JointState, '/joint_states',
            self._on_joint_states, 10,
            callback_group=self._cbg,
        )
        self.create_subscription(
            Float64MultiArray, '/arm/target_xyztheta',
            self._on_xyztheta, 10,
            callback_group=self._cbg,
        )
        self.create_subscription(
            Pose, '/arm/target_pose',
            self._on_pose, 10,
            callback_group=self._cbg,
        )

        self.get_logger().info('IK solver ready.')
        self.get_logger().info('  /arm/target_xyztheta  [x, y, z, theta]')
        self.get_logger().info('  /arm/target_pose       geometry_msgs/Pose')

    # ==============================================================
    # Subscribers
    # ==============================================================

    def _on_joint_states(self, msg: JointState) -> None:
        with self._state_lock:
            for name, pos in zip(msg.name, msg.position):
                stage = self._JOINT_MAP.get(name)
                if stage:
                    self._current_angles[stage] = float(pos)

    def _on_xyztheta(self, msg: Float64MultiArray) -> None:
        data = msg.data
        if len(data) < 3:
            self.get_logger().error('target_xyztheta needs at least [x, y, z]')
            return
        x     = float(data[0])
        y     = float(data[1])
        z     = float(data[2])
        theta = float(data[3]) if len(data) > 3 else 0.0

        self.get_logger().info(
            f'XYZ target: ({x:.4f}, {y:.4f}, {z:.4f}) '
            f'theta={math.degrees(theta):.1f}°'
        )

        with self._state_lock:
            seed = dict(self._current_angles)

        self._status('SOLVING')
        angles = self._arm.solve_down(x, y, z, theta, initial_angles=seed)

        if angles is None:
            self.get_logger().warn(f'IK: unreachable ({x:.3f},{y:.3f},{z:.3f})')
            self._status('UNREACHABLE')
            return

        self._send_angles(angles)

    def _on_pose(self, msg: Pose) -> None:
        x = msg.position.x
        y = msg.position.y
        z = msg.position.z

        qx, qy, qz, qw = (msg.orientation.x, msg.orientation.y,
                           msg.orientation.z, msg.orientation.w)
        roll  = math.atan2(2*(qw*qx + qy*qz), 1 - 2*(qx**2 + qy**2))
        pitch = math.asin(max(-1.0, min(1.0, 2*(qw*qy - qz*qx))))
        yaw   = math.atan2(2*(qw*qz + qx*qy), 1 - 2*(qy**2 + qz**2))

        self.get_logger().info(
            f'Pose target: ({x:.4f},{y:.4f},{z:.4f}) '
            f'rpy=({math.degrees(roll):.1f}°,'
            f'{math.degrees(pitch):.1f}°,'
            f'{math.degrees(yaw):.1f}°)'
        )

        with self._state_lock:
            seed = dict(self._current_angles)

        self._status('SOLVING')
        angles = self._arm.solve_pose(x, y, z, roll, pitch, yaw,
                                      initial_angles=seed)
        if angles is None:
            self.get_logger().warn('IK: no solution for pose')
            self._status('UNREACHABLE')
            return

        self._send_angles(angles)

    # ==============================================================
    # Motion
    # ==============================================================

    def _send_angles(self, angles: dict) -> None:
        if self._busy:
            self.get_logger().warn('IK: busy, ignoring new target')
            return

        cmd = angles_to_syncabs(angles)
        self.get_logger().info(f'IK → {cmd}')
        for j in ['S1','S2','S3','S4','S5']:
            if j in angles:
                self.get_logger().debug(
                    f'  {j}: {math.degrees(angles[j]):.2f}°'
                )

        self._status('SENDING')
        self._busy = True

        if not self._action_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error('IK: /arm/move not available')
            self._busy = False
            self._status('ERR')
            return

        goal = ArmMove.Goal()
        goal.command = cmd
        future = self._action_client.send_goal_async(
            goal, feedback_callback=self._on_feedback
        )
        future.add_done_callback(self._on_goal_response)

    def _on_feedback(self, feedback_msg) -> None:
        self.get_logger().debug(f'feedback: {feedback_msg.feedback.status}')

    def _on_goal_response(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('IK: goal rejected')
            self._busy = False
            self._status('ERR')
            return
        goal_handle.get_result_async().add_done_callback(self._on_result)

    def _on_result(self, future) -> None:
        result = future.result().result
        self._busy = False
        if result.success:
            self.get_logger().info(f'IK done — {result.response}')
            self._status('DONE')
        else:
            self.get_logger().warn(f'IK failed — {result.response}')
            self._status('ERR')

    def _status(self, msg: str) -> None:
        s = String()
        s.data = msg
        self._pub_status.publish(s)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = IKSolverNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()