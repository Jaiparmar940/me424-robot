"""
keyboard_control.py  —  XYZ+theta keyboard control for the ME424 arm.

Architecture:
  - Maintains a target (x, y, z, theta5) in world coordinates
  - On each keypress, updates the target and runs IK
  - Sends a single `syncabs` command to the ESP32 via the esp32_cmd topic
  - The tcp_bridge node forwards that to the ESP32 over TCP

Run alongside tcp_bridge:
  Terminal 1:  ros2 run arm_robot_bridge tcp_bridge
  Terminal 2:  ros2 run arm_robot_bridge keyboard_control

Controls:
  Arrow keys / WASD  →  move X/Y
  Q / E              →  move Z up/down
  Z / C              →  rotate S5 (wrist twist)
  [ / ]              →  decrease/increase step size
  H                  →  home all axes (sends home sequence)
  SPACE              →  stop / e-stop
  ESC or X           →  exit
"""

import math
import sys
import termios
import tty
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# ─────────────────────────────────────────────────────────────
# Robot geometry  (metres)
# ─────────────────────────────────────────────────────────────
INCH = 0.0254

L1   = 6.335 * INCH   # S1→S2 (vertical riser, always fixed)
L2   = 10.0  * INCH   # S2→S3
L3   = 8.0   * INCH   # S3→S4
L4   = 3.209 * INCH   # S4→S5
L5   = 1.143 * INCH   # S5→connector end
LARM = L4 + L5        # total wrist length

# ─────────────────────────────────────────────────────────────
# Steps per revolution for each stage (after gearbox / pulleys)
# Index matches syncabs argument order:
#   t1=C1=STAGE2_RIGHT  t2=C2=STAGE3  t3=C3=STAGE4
#   t4=C4=STAGE2_LEFT   t5=C5=STAGE5  t6=C6=STAGE1
# ─────────────────────────────────────────────────────────────
STEPS_PER_REV = {
    'S1': 1040,   # turntable      → C6 → syncabs index 5
    'S2': 3600,   # base lift      → C1+C4 → syncabs index 0 (and 3, kept equal)
    'S3': 2400,   # arm segment    → C2 → syncabs index 1
    'S4': 2100,   # arm segment    → C3 → syncabs index 2
    'S5': 200,    # wrist twist    → C5 → syncabs index 4
}

def steps_per_rad(stage):
    return STEPS_PER_REV[stage] / (2.0 * math.pi)

# ─────────────────────────────────────────────────────────────
# Joint zero-point offsets
#
# The IK solves in "math convention":
#   S2_math=0  → arm horizontal toward +X
#   S3_math=0  → forearm straight (continuation of S2)
#   S4_math=0  → wrist straight (continuation of S3)
#
# Physical zeros (from dimensions file):
#   S2_phys=0  → arm pointing straight up
#   S3_phys=0  → straight up (negative lowers toward +X at S2=0)
#   S4_phys=0  → straight up
#
# Conversion: phys = math - offset
# ─────────────────────────────────────────────────────────────
S2_OFFSET = math.radians(90)   # math horizontal → physical vertical
S3_OFFSET = math.pi            # elbow flips sign convention
S4_OFFSET = math.pi            # wrist flips sign convention

# ─────────────────────────────────────────────────────────────
# Joint limits (physical convention, radians)
# ─────────────────────────────────────────────────────────────
LIMITS = {
    'S1': (math.radians(-129), math.radians(156)),
    'S2': (math.radians(-94.16), math.radians(86)),
    'S3': (math.radians(-113),   math.radians(79.6)),
    'S4': (math.radians(-83),    math.radians(79)),
    'S5': (math.radians(-180),   math.radians(180)),
}

# ─────────────────────────────────────────────────────────────
# syncabs controller order (6 values):
#   index 0 = C1 = STAGE2_RIGHT
#   index 1 = C2 = STAGE3
#   index 2 = C3 = STAGE4
#   index 3 = C4 = STAGE2_LEFT  (always equals index 0)
#   index 4 = C5 = STAGE5
#   index 5 = C6 = STAGE1
# ─────────────────────────────────────────────────────────────
SYNCABS_MAX_SPS   = 800    # steps/sec at full speed
SYNCABS_RAMP_STEPS = 200   # ramp duration in master steps

# ─────────────────────────────────────────────────────────────
# Workspace / step sizes
# ─────────────────────────────────────────────────────────────
STEP_SIZES_M   = [0.002, 0.005, 0.010, 0.025]   # XY/Z step options (metres)
STEP_SIZES_RAD = [math.radians(1), math.radians(2),
                  math.radians(5), math.radians(10)]  # theta5 step options
DEFAULT_STEP_IDX = 1   # start at 5mm / 2°

# Starting position — arm pointing straight down above origin
# Adjust to a safe known position for your physical arm
START_X =  0.10   # 10 cm in front of base
START_Y =  0.0
START_Z =  0.20   # 20 cm above table
START_T5 = 0.0    # wrist twist = 0


# ═════════════════════════════════════════════════════════════
# Inverse Kinematics
# ═════════════════════════════════════════════════════════════

def inverse_kinematics(x, y, z, theta5=0.0):
    """
    Compute joint angles (radians, physical convention) to place
    the connector end at world (x, y, z) pointing straight down.

    Returns dict {S1..S5} or None if unreachable / out of limits.
    """
    # ── Subtract wrist length: find S4 position ──────────────
    # Arm points straight down so wrist hangs directly below S4
    wx, wy, wz = x, y, z + LARM

    # ── S1: base rotation ─────────────────────────────────────
    theta1 = math.atan2(wy, wx)

    # ── Project into the arm's vertical plane ─────────────────
    r = math.sqrt(wx**2 + wy**2)   # horizontal reach
    h = wz - L1                     # height of S4 above S2

    d = math.sqrt(r**2 + h**2)     # S2→S4 straight-line distance

    max_reach = L2 + L3
    min_reach = abs(L2 - L3)

    if d > max_reach:
        return None, f"unreachable: d={d*100:.1f}cm > max {max_reach*100:.1f}cm"
    if d < min_reach:
        return None, f"unreachable: d={d*100:.1f}cm < min {min_reach*100:.1f}cm"

    # ── S3 (elbow) via law of cosines — elbow-up solution ─────
    cos_s3 = (d**2 - L2**2 - L3**2) / (2.0 * L2 * L3)
    cos_s3 = max(-1.0, min(1.0, cos_s3))
    s3_math = math.acos(cos_s3)

    # ── S2 (shoulder) ─────────────────────────────────────────
    alpha = math.atan2(h, r)
    beta  = math.acos(
        max(-1.0, min(1.0, (L2**2 + d**2 - L3**2) / (2.0 * L2 * d)))
    )
    s2_math = alpha + beta

    # ── S4 (wrist pitch) — keep arm pointing straight down ────
    # Total angle chain must sum to -90° from horizontal
    s4_math = -(math.pi / 2.0) - s2_math - s3_math

    # ── Convert to physical joint conventions ─────────────────
    s2_phys = s2_math - S2_OFFSET
    s3_phys = -(s3_math - math.pi)   # elbow convention flip
    s4_phys = -(s4_math + math.pi)   # wrist convention flip

    angles = {
        'S1': theta1,
        'S2': s2_phys,
        'S3': s3_phys,
        'S4': s4_phys,
        'S5': theta5,
    }

    # ── Check limits ──────────────────────────────────────────
    violations = []
    for joint, angle in angles.items():
        lo, hi = LIMITS[joint]
        if angle < lo or angle > hi:
            violations.append(
                f"{joint}: {math.degrees(angle):.1f}° "
                f"(limit [{math.degrees(lo):.1f}°, {math.degrees(hi):.1f}°])"
            )

    if violations:
        return None, "limit violations: " + ", ".join(violations)

    return angles, None


def angles_to_syncabs(angles, current_steps):
    """
    Convert joint angles (radians) to a syncabs command string.
    current_steps: dict {S1..S5} tracking the ESP32's currentPos.

    syncabs t1 t2 t3 t4 t5 t6 max_sps ramp_steps
      t1 = C1 = STAGE2_RIGHT
      t2 = C2 = STAGE3
      t3 = C3 = STAGE4
      t4 = C4 = STAGE2_LEFT  (= t1)
      t5 = C5 = STAGE5
      t6 = C6 = STAGE1
    """
    t = {}
    for stage in ['S1', 'S2', 'S3', 'S4', 'S5']:
        t[stage] = int(angles[stage] * steps_per_rad(stage))

    # Build the 6-value syncabs string
    t1 = t['S2']   # C1 = STAGE2_RIGHT
    t2 = t['S3']   # C2 = STAGE3
    t3 = t['S4']   # C3 = STAGE4
    t4 = t['S2']   # C4 = STAGE2_LEFT  (must equal t1)
    t5 = t['S5']   # C5 = STAGE5
    t6 = t['S1']   # C6 = STAGE1

    cmd = (f"syncabs {t1} {t2} {t3} {t4} {t5} {t6} "
           f"{SYNCABS_MAX_SPS} {SYNCABS_RAMP_STEPS}")
    return cmd, t


# ═════════════════════════════════════════════════════════════
# Terminal key reading
# ═════════════════════════════════════════════════════════════

def get_key():
    """Read one keypress (or escape sequence) from stdin."""
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
        if ch == '\x1b':
            # Arrow key escape sequence: ESC [ A/B/C/D
            try:
                ch2 = sys.stdin.read(1)
                ch3 = sys.stdin.read(1)
                if ch2 == '[':
                    return {'A': 'UP', 'B': 'DOWN',
                            'C': 'RIGHT', 'D': 'LEFT'}.get(ch3, 'ESC')
            except Exception:
                pass
            return 'ESC'
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
    return ch


# ═════════════════════════════════════════════════════════════
# ROS2 Node
# ═════════════════════════════════════════════════════════════

class KeyboardControl(Node):
    def __init__(self):
        super().__init__('keyboard_control')

        self.pub = self.create_publisher(String, 'esp32_cmd', 10)
        self.sub = self.create_subscription(
            String, 'esp32_feedback', self.feedback_cb, 10)

        # Current target position
        self.x  = START_X
        self.y  = START_Y
        self.z  = START_Z
        self.t5 = START_T5

        # Step size index
        self.step_idx = DEFAULT_STEP_IDX

        # Tracked absolute step positions (mirrors ESP32 currentPos)
        # These are updated every time we send a syncabs command.
        # Run `home` first to establish true zero.
        self.current_steps = {'S1': 0, 'S2': 0, 'S3': 0, 'S4': 0, 'S5': 0}

        self.get_logger().info("Keyboard control ready — press H to home first")

    def send(self, cmd):
        msg = String()
        msg.data = cmd
        self.pub.publish(msg)

    def feedback_cb(self, msg):
        """Print ESP32 feedback to terminal."""
        line = msg.data.strip()
        if line:
            print(f"\r  [ESP32] {line}")
            self._reprint_status()

    def _reprint_status(self):
        step_m   = STEP_SIZES_M[self.step_idx]
        step_deg = math.degrees(STEP_SIZES_RAD[self.step_idx])
        print(
            f"\r  pos=({self.x*100:.1f}, {self.y*100:.1f}, {self.z*100:.1f}) cm  "
            f"θ5={math.degrees(self.t5):.1f}°  "
            f"step={step_m*1000:.1f}mm/{step_deg:.1f}°  ",
            end='', flush=True
        )

    def move_to(self, x, y, z, t5):
        """Run IK and send syncabs if solution found."""
        angles, err = inverse_kinematics(x, y, z, t5)
        if angles is None:
            print(f"\r  [IK] {err}  ", flush=True)
            return False

        cmd, new_steps = angles_to_syncabs(angles, self.current_steps)

        # Update tracked state
        self.x, self.y, self.z, self.t5 = x, y, z, t5
        self.current_steps = {
            'S1': new_steps['S1'],
            'S2': new_steps['S2'],
            'S3': new_steps['S3'],
            'S4': new_steps['S4'],
            'S5': new_steps['S5'],
        }

        self.send(cmd)
        self._reprint_status()
        return True

    def print_controls(self):
        print("""
╔══════════════════════════════════════════════════════╗
║           ME424 Arm  —  XYZ Keyboard Control         ║
╠══════════════════════════════════════════════════════╣
║  Arrow keys / W A S D  →  X/Y (horizontal)          ║
║  Q / E                 →  Z up / down                ║
║  Z / C                 →  S5 wrist twist CCW / CW    ║
║  [ / ]                 →  step size smaller/larger   ║
║  H                     →  home all axes              ║
║  SPACE                 →  e-stop                     ║
║  ESC or X              →  exit                       ║
╚══════════════════════════════════════════════════════╝
  ⚠  Run  home s2, home s3, home s4, home s5  first!
""")

    def run(self):
        self.print_controls()
        self._reprint_status()

        while True:
            key = get_key()

            step_m   = STEP_SIZES_M[self.step_idx]
            step_rad = STEP_SIZES_RAD[self.step_idx]

            nx, ny, nz, nt5 = self.x, self.y, self.z, self.t5

            # ── XY movement ───────────────────────────────────
            if key in ('w', 'UP'):
                nx += step_m
            elif key in ('s', 'DOWN'):
                nx -= step_m
            elif key in ('a', 'LEFT'):
                ny += step_m
            elif key in ('d', 'RIGHT'):
                ny -= step_m

            # ── Z movement ────────────────────────────────────
            elif key == 'q':
                nz += step_m
            elif key == 'e':
                nz -= step_m

            # ── Wrist twist ───────────────────────────────────
            elif key == 'z':
                nt5 += step_rad
            elif key == 'c':
                nt5 -= step_rad

            # ── Step size ─────────────────────────────────────
            elif key == '[':
                self.step_idx = max(0, self.step_idx - 1)
                self._reprint_status()
                continue
            elif key == ']':
                self.step_idx = min(len(STEP_SIZES_M) - 1, self.step_idx + 1)
                self._reprint_status()
                continue

            # ── Home ──────────────────────────────────────────
            elif key == 'h':
                print("\r  Homing all axes...                    ")
                self.send("home s2")
                self.send("home s3")
                self.send("home s4")
                self.send("home s5")
                self.send("home s1")
                self.current_steps = {'S1': 0, 'S2': 0, 'S3': 0, 'S4': 0, 'S5': 0}
                self.x, self.y, self.z, self.t5 = START_X, START_Y, START_Z, 0.0
                print("  Homed. Moving to start position...")
                self.move_to(self.x, self.y, self.z, self.t5)
                continue

            # ── E-stop ────────────────────────────────────────
            elif key == ' ':
                print("\r  *** E-STOP ***                        ")
                self.send("estop")
                continue

            # ── Exit ──────────────────────────────────────────
            elif key in ('x', 'ESC', '\x03'):
                print("\r  Exiting...                            ")
                break

            else:
                continue

            # ── Attempt move ──────────────────────────────────
            self.move_to(nx, ny, nz, nt5)


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardControl()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import String
# import sys
# import termios
# import tty


# STEP_SIZE = 50  # steps per key press


# class KeyboardControl(Node):
#     def __init__(self):
#         super().__init__('keyboard_control')

#         self.pub = self.create_publisher(String, 'esp32_cmd', 10)

#         self.get_logger().info("Keyboard control ready")

#     def send(self, cmd):
#         msg = String()
#         msg.data = cmd
#         self.pub.publish(msg)
#         print(f"Sent: {cmd}")

#     def run(self):
#         print("""
# Controls:
#  q/a → base
#  w/s → shoulder
#  e/d → elbow
#  r/f → wrist pitch
#  t/g → wrist twist

#  x → exit
# """)

#         while True:
#             key = get_key()

#             if key == 'q':
#                 self.send(f"s1cw {STEP_SIZE}")
#             elif key == 'a':
#                 self.send(f"s1ccw {STEP_SIZE}")

#             elif key == 'w':
#                 self.send(f"s2up {STEP_SIZE}")
#             elif key == 's':
#                 self.send(f"s2down {STEP_SIZE}")

#             elif key == 'e':
#                 self.send(f"s3up {STEP_SIZE}")
#             elif key == 'd':
#                 self.send(f"s3down {STEP_SIZE}")

#             elif key == 'r':
#                 self.send(f"s4up {STEP_SIZE}")
#             elif key == 'f':
#                 self.send(f"s4down {STEP_SIZE}")

#             elif key == 't':
#                 self.send(f"s5cw {STEP_SIZE}")
#             elif key == 'g':
#                 self.send(f"s5ccw {STEP_SIZE}")

#             elif key == 'x':
#                 break


# def get_key():
#     fd = sys.stdin.fileno()
#     old = termios.tcgetattr(fd)
#     try:
#         tty.setraw(fd)
#         ch = sys.stdin.read(1)
#     finally:
#         termios.tcsetattr(fd, termios.TCSADRAIN, old)
#     return ch


# def main(args=None):
#     rclpy.init(args=args)
#     node = KeyboardControl()

#     try:
#         node.run()
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()