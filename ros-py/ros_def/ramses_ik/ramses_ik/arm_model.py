"""
arm_model.py
============
Pure-Python wrapper around ikpy for the Ramses 5-DOF arm.
No ROS dependency — can be tested standalone.

Coordinate system (from robot_dimensions.txt)
---------------------------------------------
Origin  : centre of turntable, top surface of docking station
+X axis : from turntable towards docking stations
+Z axis : upward
Units   : metres

Joint convention
----------------
All joints are revolute. Zero position = all stages at their
homed/zeroed position as defined in robot_dimensions.txt.

  S1 (turntable)  : rotates about Z. 0° = arm pointing towards docking station (+X)
  S2 (shoulder)   : rotates about horizontal axis. 0° = arm pointing straight up
  S3 (elbow)      : rotates about horizontal axis. 0° = arm pointing straight up
  S4 (wrist)      : rotates about horizontal axis. 0° = arm pointing straight up
  S5 (wrist twist): rotates about the L4/L5 link axis. 0° = default orientation

Steps per revolution (16x microstepping, from robot_dimensions.txt)
--------------------------------------------------------------------
  S1: 16640   S2: 57600   S3: 38400   S4: 33600   S5: 3200
"""

import math
import os
from typing import List, Optional, Tuple

import numpy as np

# ikpy is imported lazily so this module can be syntax-checked without it.
try:
    import ikpy.chain
    import ikpy.link
    _IKPY_AVAILABLE = True
except ImportError:
    _IKPY_AVAILABLE = False

# ---------------------------------------------------------------------------
# Physical constants
# ---------------------------------------------------------------------------

INCH = 0.0254  # metres per inch

# Link lengths (metres)
L1 = 6.335 * INCH   # S1 → S2  (vertical riser)
L2 = 10.0  * INCH   # S2 → S3
L3 = 8.0   * INCH   # S3 → S4
L4 = 3.209 * INCH   # S4 → S5
L5 = 1.143 * INCH   # S5 → connector tip

# Docking station positions (metres, from origin)
DOCK_PROBE    = (9.515 * INCH,  4.434 * INCH, 0.0)
DOCK_BONESAW  = (10.5  * INCH,  0.0,          0.0)
DOCK_VACUUM   = (9.515 * INCH, -4.434 * INCH, 0.0)

# Tool offsets from connector tip (metres, in tool frame)
# Tool frame: Z = outward from connector, X = right, Y = up along connector
TOOL_PROBE   = (0.0,        0.0,        5.350 * INCH)
TOOL_BONESAW = (0.0,        4.705 * INCH, 2.197 * INCH)
TOOL_VACUUM  = (0.0,        0.0,        2.635 * INCH)

# Joint limits (radians, from robot_dimensions.txt)
JOINT_LIMITS = {
    'S1': (-129.0, 156.0),
    'S2': (-94.16,  86.0),
    'S3': (-113.0,  79.6),
    'S4': (-83.0,   79.0),
    'S5': (-180.0, 180.0),
}
JOINT_LIMITS_RAD = {
    k: (math.radians(lo), math.radians(hi))
    for k, (lo, hi) in JOINT_LIMITS.items()
}

# Steps per full output revolution (8x microstepping)
MICROSTEPS_PER_REV = {
    'S1':  8320,   # S1 turntable   (1040 full steps × 8)
    'S2': 28800,   # S2 shoulder    (3600 full steps × 8)
    'S3': 19200,   # S3 elbow       (2400 full steps × 8)
    'S4': 16800,   # S4 wrist       (2100 full steps × 8)
    'S5':  1600,   # S5 wrist twist ( 200 full steps × 8)
}

# ---------------------------------------------------------------------------
# Firmware zero = autohome position = your defined physical zero
# ---------------------------------------------------------------------------
# After autohome, all step counts = 0 at the defined zero positions:
#   S1: arm pointing towards docking station (+X)
#   S2: arm straight up
#   S3: arm straight up
#   S4: arm straight up
#   S5: default wrist orientation
#
# So the conversion is simply:
#   physical_angle = steps × rad_per_step
#   steps = physical_angle / rad_per_step
#
# AXIS_SIGN: +1 if positive steps = positive angle, -1 if inverted.
# Verify on physical arm after autohome by jogging each axis.
AXIS_SIGN = {
    'S1': +1,
    'S2': +1,
    'S3': +1,
    'S4': +1,
    'S5': +1,
}

# URDF/ikpy frame vs physical frame offset.
# ikpy's zero for each joint (from Onshape) may differ from your physical zero.
# Run: ros2 run ramses_ik calibrate  to determine these values.
# urdf_angle = physical_angle + JOINT_ZERO_OFFSETS[joint]
JOINT_ZERO_OFFSETS = {
    'S1': 0.0,
    'S2': 0.0,
    'S3': 0.0,
    'S4': 0.0,
    'S5': 0.0,
}


# ---------------------------------------------------------------------------
# Angle ↔ step conversion
# ---------------------------------------------------------------------------

def steps_to_physical(joint: str, steps: int) -> float:
    """Convert firmware step count to physical angle (radians from autohome zero)."""
    rad_per_step = (2.0 * math.pi) / MICROSTEPS_PER_REV[joint]
    return AXIS_SIGN[joint] * steps * rad_per_step


def physical_to_steps(joint: str, physical_rad: float) -> int:
    """Convert physical angle (radians) to firmware step count. Clamps to joint limits."""
    lo, hi = JOINT_LIMITS_RAD[joint]
    clamped = max(lo, min(hi, physical_rad))
    rad_per_step = (2.0 * math.pi) / MICROSTEPS_PER_REV[joint]
    return round(AXIS_SIGN[joint] * clamped / rad_per_step)


def physical_to_urdf(joint: str, physical_rad: float) -> float:
    """Convert physical angle to URDF/ikpy frame."""
    return physical_rad + JOINT_ZERO_OFFSETS[joint]


def urdf_to_physical(joint: str, urdf_rad: float) -> float:
    """Convert URDF/ikpy angle to physical angle."""
    return urdf_rad - JOINT_ZERO_OFFSETS[joint]


def radians_to_steps(joint: str, physical_rad: float) -> int:
    """Alias for physical_to_steps."""
    return physical_to_steps(joint, physical_rad)


def steps_to_radians(joint: str, steps: int) -> float:
    """Alias for steps_to_physical."""
    return steps_to_physical(joint, steps)


def joint_angles_to_steps(angles: dict) -> dict:
    """Convert {joint: urdf_angle_rad} to {joint: firmware_steps}."""
    return {
        joint: physical_to_steps(joint, urdf_to_physical(joint, urdf_rad))
        for joint, urdf_rad in angles.items()
    }


def steps_to_urdf_angles(steps_dict: dict) -> dict:
    """Convert {joint: firmware_steps} to {joint: urdf_angle_rad}."""
    return {
        joint: physical_to_urdf(joint, steps_to_physical(joint, steps))
        for joint, steps in steps_dict.items()
    }


def angles_to_syncabs(angles: dict, max_sps: int = 800, ramp: int = 40) -> str:
    """
    Convert IK solution angles (URDF frame, radians) to a syncabs command.
    Applies zero offsets so the firmware gets physical step counts.

    syncabs controller order: t1=C1(S2R) t2=C2(S3) t3=C3(S4) t4=C4(S2L) t5=C5(S5) t6=C6(S1)
    """
    steps = joint_angles_to_steps(angles)
    s1 = steps.get('S1', 0)
    s2 = steps.get('S2', 0)
    s3 = steps.get('S3', 0)
    s4 = steps.get('S4', 0)
    s5 = steps.get('S5', 0)
    return f"syncabs {s2} {s3} {s4} {s2} {s5} {s1} {max_sps} {ramp}"


def firmware_pos_to_urdf_angles(pos_line: str) -> dict:
    """
    Parse a firmware POS line and return URDF-frame joint angles.
    Input:  'POS C1=10200 C2=0 C3=0 C4=10200 C5=0 C6=0'
    Output: {'S1': rad, 'S2': rad, 'S3': rad, 'S4': rad, 'S5': rad}

    Controller → stage mapping:
      C1 = S2 (right),  C2 = S3,  C3 = S4,  C4 = S2 (left, mirror)
      C5 = S5,          C6 = S1
    """
    import re
    values = {}
    for match in re.finditer(r'C(\d+)=(-?\d+)', pos_line):
        c = int(match.group(1))
        steps = int(match.group(2))
        if c == 1:
            values['S2'] = steps
        elif c == 2:
            values['S3'] = steps
        elif c == 3:
            values['S4'] = steps
        elif c == 5:
            values['S5'] = steps
        elif c == 6:
            values['S1'] = steps
        # C4 mirrors C1, ignored

    return steps_to_urdf_angles(values)


# ---------------------------------------------------------------------------
# Calibration helper
# ---------------------------------------------------------------------------

def print_calibration_report(arm: 'ArmModel') -> None:
    """
    Print FK output at all-zero joint angles (URDF frame).
    Use this to determine JOINT_ZERO_OFFSETS:
      Home the arm physically, then run this — the FK output tells you
      where ikpy thinks the end effector is at firmware zero.
      Adjust JOINT_ZERO_OFFSETS until FK output matches physical reality.
    """
    print('\n=== Calibration report ===')
    print('FK at all joints = 0 (URDF frame, before any offsets):')
    zeros = {j: 0.0 for j in ['S1', 'S2', 'S3', 'S4', 'S5']}
    fk = arm.forward_kinematics(zeros)
    x, y, z = fk[0, 3], fk[1, 3], fk[2, 3]
    print(f'  End effector: x={x:.4f}  y={y:.4f}  z={z:.4f} (metres)')
    print()
    print('Current JOINT_ZERO_OFFSETS:')
    for j, v in JOINT_ZERO_OFFSETS.items():
        print(f'  {j}: {v:.4f} rad  ({math.degrees(v):.2f}°)')
    print()
    print('To calibrate:')
    print('  1. Home all axes on the physical robot')
    print('  2. Measure actual end-effector position')
    print('  3. Adjust JOINT_ZERO_OFFSETS in arm_model.py until')
    print('     FK output matches your measurement')
    print('  4. Rebuild: colcon build --packages-select ramses_ik')
    print('=====================================\n')


# ---------------------------------------------------------------------------
# IK solver (ikpy-based)
# ---------------------------------------------------------------------------

class ArmModel:
    """
    Wraps ikpy to solve IK for the Ramses arm.

    The chain is built from the URDF file so geometry is always consistent
    with RViz2. ikpy needs a URDF with named active links.
    """

    # ikpy includes a dummy base link and a dummy end link in the chain,
    # so the joint indices are offset by 1.
    _ACTIVE_LINKS = [
        's1_turntable',
        's2_shoulder',
        's3_elbow',
        's4_wrist',
        's5_wrist_twist',
    ]

    def __init__(self, urdf_path: str) -> None:
        if not _IKPY_AVAILABLE:
            raise ImportError(
                'ikpy is not installed. Run: pip install ikpy --break-system-packages'
            )
        if not os.path.exists(urdf_path):
            raise FileNotFoundError(f'URDF not found: {urdf_path}')

        self._chain = ikpy.chain.Chain.from_urdf_file(
            urdf_path,
            base_elements=['b_turntablebase_001'],
            active_links_mask=self._build_active_mask(urdf_path),
        )
        self._urdf_path = urdf_path

    def _build_active_mask(self, urdf_path: str) -> List[bool]:
        """
        Build the active_links_mask for ikpy.
        ikpy adds a base link and end link around the chain, so the mask
        is [False, True, True, True, True, True, False].
        """
        # Parse joint count from URDF
        import xml.etree.ElementTree as ET
        tree = ET.parse(urdf_path)
        root = tree.getroot()
        joint_names = [
            j.get('name') for j in root.findall('joint')
            if j.get('type') != 'fixed'
        ]
        n = len(joint_names)
        # ikpy chain has n+2 links (base + joints + end effector)
        mask = [False] + [True] * n + [False]
        return mask

    def forward_kinematics(self, angles: dict) -> np.ndarray:
        """
        Compute end-effector pose from joint angles.
        angles: {'S1': rad, 'S2': rad, ...}
        Returns 4×4 homogeneous transform matrix.
        """
        joint_list = self._angles_to_ikpy_list(angles)
        return self._chain.forward_kinematics(joint_list)

    def end_effector_position(self, angles: dict) -> Tuple[float, float, float]:
        """Return (x, y, z) of end effector in metres."""
        fk = self.forward_kinematics(angles)
        return float(fk[0, 3]), float(fk[1, 3]), float(fk[2, 3])

    def solve_down(
        self,
        x: float,
        y: float,
        z: float,
        theta: float = 0.0,
        initial_angles: Optional[dict] = None,
    ) -> Optional[dict]:
        """
        Solve IK for end effector at (x, y, z) pointing straight down,
        with S5 twist = theta radians.

        'Pointing straight down' means the end-effector Z axis aligns with
        world -Z. This is enforced by fixing the target orientation to a
        downward-pointing rotation matrix.

        Returns dict of {joint: angle_rad} or None if no solution found.
        """
        # Target orientation: end effector pointing straight down
        # In the arm's frame, this means the tool Z axis points along world -Z.
        target_position = np.array([x, y, z])
        target_orientation = np.array([
            [1,  0,  0],
            [0, -1,  0],
            [0,  0, -1],
        ])

        initial = self._angles_to_ikpy_list(initial_angles or {})

        try:
            solution = self._chain.inverse_kinematics(
                target_position,
                target_orientation,
                orientation_mode='all',
                initial_position=initial,
            )
        except Exception:
            # Try without orientation constraint as fallback
            try:
                solution = self._chain.inverse_kinematics(
                    target_position,
                    initial_position=initial,
                )
            except Exception:
                return None

        angles = self._ikpy_list_to_angles(solution)
        if angles is None:
            return None

        # Override S5 with requested twist
        angles['S5'] = theta

        # Validate all joints within limits
        for joint, angle in angles.items():
            lo, hi = JOINT_LIMITS_RAD[joint]
            if angle < lo - 0.05 or angle > hi + 0.05:
                return None  # Out of reach

        return angles

    def solve_pose(
        self,
        x: float,
        y: float,
        z: float,
        roll: float = 0.0,
        pitch: float = 0.0,
        yaw: float = 0.0,
        initial_angles: Optional[dict] = None,
    ) -> Optional[dict]:
        """
        Full IK solve for arbitrary end-effector pose.
        Orientation specified as roll/pitch/yaw (radians, intrinsic XYZ).
        Returns dict of {joint: angle_rad} or None if no solution found.
        """
        target_position = np.array([x, y, z])

        # Build rotation matrix from RPY
        cr, sr = math.cos(roll),  math.sin(roll)
        cp, sp = math.cos(pitch), math.sin(pitch)
        cy, sy = math.cos(yaw),   math.sin(yaw)
        target_orientation = np.array([
            [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
            [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
            [-sp,   cp*sr,            cp*cr           ],
        ])

        initial = self._angles_to_ikpy_list(initial_angles or {})

        try:
            solution = self._chain.inverse_kinematics(
                target_position,
                target_orientation,
                orientation_mode='all',
                initial_position=initial,
            )
        except Exception:
            return None

        return self._ikpy_list_to_angles(solution)

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _angles_to_ikpy_list(self, angles: dict) -> List[float]:
        """
        Convert {joint: rad} dict to ikpy's flat list format.
        ikpy expects [base_dummy, s1, s2, s3, s4, s5, end_dummy].
        """
        order = ['S1', 'S2', 'S3', 'S4', 'S5']
        return [0.0] + [angles.get(j, 0.0) for j in order] + [0.0]

    def _ikpy_list_to_angles(self, solution: List[float]) -> Optional[dict]:
        """Convert ikpy flat list back to {joint: rad} dict."""
        if solution is None or len(solution) < 6:
            return None
        order = ['S1', 'S2', 'S3', 'S4', 'S5']
        return {j: float(solution[i + 1]) for i, j in enumerate(order)}
