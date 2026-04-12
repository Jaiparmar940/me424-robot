"""
teleop_keyboard.py
==================
Keyboard teleoperation node for the Ramses arm.
Jogs the end-effector in XYZ and controls S5 wrist twist.
Publishes to /arm/target_xyztheta which the IK solver picks up.

Controls
--------
  W / S        : +Y / -Y  (forward/back in workspace)
  A / D        : -X / +X  (left/right in workspace)
  Q / E        : +Z / -Z  (up/down)
  J / L        : S5 twist CCW / CW
  [ / ]        : decrease / increase step size
  H            : print help
  Space        : send current position immediately (no jog)
  Ctrl+C       : quit

Run on the Pi (or laptop if ROS2 is installed there):
  ros2 run ramses_ik teleop_keyboard
"""

import math
import sys
import termios
import tty
from typing import Tuple

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


# Default starting position (metres) — arm above centre of workspace
DEFAULT_X = 0.15
DEFAULT_Y = 0.0
DEFAULT_Z = 0.10
DEFAULT_THETA = 0.0  # S5 twist radians

# Step sizes (metres for XYZ, radians for theta)
STEP_SIZES_XYZ   = [0.005, 0.01, 0.02, 0.05]  # metres
STEP_SIZES_THETA = [0.05, 0.1, 0.2]            # radians
DEFAULT_STEP_IDX = 1


HELP_TEXT = """
─────────────────────────────────────────
  Ramses Arm Keyboard Teleop
─────────────────────────────────────────
  W / S    →  +Y / -Y   (fwd / back)
  A / D    →  -X / +X   (left / right)
  Q / E    →  +Z / -Z   (up / down)
  J / L    →  S5 twist CCW / CW
  [ / ]    →  step size smaller / larger
  H        →  show this help
  Space    →  resend current position
  Ctrl+C   →  quit
─────────────────────────────────────────
"""


def get_key(settings) -> str:
    """Read a single keypress without waiting for Enter."""
    tty.setraw(sys.stdin.fileno())
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


class TeleopKeyboardNode(Node):

    def __init__(self) -> None:
        super().__init__('teleop_keyboard')

        self._pub = self.create_publisher(
            Float64MultiArray, '/arm/target_xyztheta', 10
        )

        self._x     = DEFAULT_X
        self._y     = DEFAULT_Y
        self._z     = DEFAULT_Z
        self._theta = DEFAULT_THETA

        self._xyz_step_idx   = DEFAULT_STEP_IDX
        self._theta_step_idx = 0

        self.get_logger().info(HELP_TEXT)
        self._print_state()

    @property
    def _xyz_step(self) -> float:
        return STEP_SIZES_XYZ[self._xyz_step_idx]

    @property
    def _theta_step(self) -> float:
        return STEP_SIZES_THETA[self._theta_step_idx]

    def _print_state(self) -> None:
        print(
            f'\r  pos: x={self._x:+.4f}  y={self._y:+.4f}  z={self._z:+.4f}'
            f'  θ={math.degrees(self._theta):+.1f}°'
            f'  step_xyz={self._xyz_step*1000:.0f}mm'
            f'  step_θ={math.degrees(self._theta_step):.1f}°',
            end='', flush=True
        )

    def _publish(self) -> None:
        msg = Float64MultiArray()
        msg.data = [self._x, self._y, self._z, self._theta]
        self._pub.publish(msg)

    def run(self) -> None:
        """Main loop — reads keys and publishes targets."""
        settings = termios.tcgetattr(sys.stdin)
        print(HELP_TEXT)
        self._print_state()

        try:
            while rclpy.ok():
                key = get_key(settings)

                if key == '\x03':  # Ctrl+C
                    break
                elif key in ('w', 'W'):
                    self._y += self._xyz_step
                elif key in ('s', 'S'):
                    self._y -= self._xyz_step
                elif key in ('d', 'D'):
                    self._x += self._xyz_step
                elif key in ('a', 'A'):
                    self._x -= self._xyz_step
                elif key in ('q', 'Q'):
                    self._z += self._xyz_step
                elif key in ('e', 'E'):
                    self._z -= self._xyz_step
                elif key in ('j', 'J'):
                    self._theta += self._theta_step
                elif key in ('l', 'L'):
                    self._theta -= self._theta_step
                elif key == '[':
                    self._xyz_step_idx = max(0, self._xyz_step_idx - 1)
                    self._theta_step_idx = max(0, self._theta_step_idx - 1)
                    self._print_state()
                    continue
                elif key == ']':
                    self._xyz_step_idx = min(
                        len(STEP_SIZES_XYZ) - 1, self._xyz_step_idx + 1
                    )
                    self._theta_step_idx = min(
                        len(STEP_SIZES_THETA) - 1, self._theta_step_idx + 1
                    )
                    self._print_state()
                    continue
                elif key in ('h', 'H'):
                    print(HELP_TEXT)
                    self._print_state()
                    continue
                elif key == ' ':
                    pass  # resend current position
                else:
                    continue  # ignore unknown keys

                # Clamp Z to a safe minimum (don't crash into table)
                self._z = max(0.01, self._z)

                self._print_state()
                self._publish()

        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
            print('\nTeleop stopped.')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TeleopKeyboardNode()
    try:
        node.run()
    except Exception as e:
        print(f'Error: {e}')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()