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

# Steps per full output revolution (16x microstepping)
MICROSTEPS_PER_REV = {
    'S1': 8320,
    'S2': 28800,
    'S3': 19200,
    'S4': 16800,
    'S5':  1600,
}

# Mapping from joint name to controller index (0-based, matches firmware C1-C6)
# C1/C4 = S2 (paired), C2 = S3, C3 = S4, C5 = S5, C6 = S1
JOINT_TO_CONTROLLER = {
    'S1': 5,   # C6
    'S2': 0,   # C1 (C4 mirrors automatically in firmware)
    'S3': 1,   # C2
    'S4': 2,   # C3
    'S5': 4,   # C5
}


# ---------------------------------------------------------------------------
# Angle ↔ step conversion
# ---------------------------------------------------------------------------

def radians_to_steps(joint: str, angle_rad: float) -> int:
    """Convert a joint angle in radians to absolute step count from zero."""
    steps_per_rev = MICROSTEPS_PER_REV[joint]
    return round(angle_rad * steps_per_rev / (2.0 * math.pi))


def steps_to_radians(joint: str, steps: int) -> float:
    """Convert an absolute step count to radians."""
    steps_per_rev = MICROSTEPS_PER_REV[joint]
    return steps * (2.0 * math.pi) / steps_per_rev


def joint_angles_to_steps(angles: dict) -> dict:
    """
    Convert a dict of {joint_name: angle_rad} to {joint_name: steps}.
    Clamps to joint limits before converting.
    """
    result = {}
    for joint, angle in angles.items():
        lo, hi = JOINT_LIMITS_RAD[joint]
        clamped = max(lo, min(hi, angle))
        result[joint] = radians_to_steps(joint, clamped)
    return result


def angles_to_syncabs(angles: dict) -> str:
    """
    Convert joint angles (radians) to a syncabs firmware command string.
    Returns e.g. "syncabs 0 400 -200 100 0 0 800 40"

    Controller order for syncabs: t1=C1 t2=C2 t3=C3 t4=C4 t5=C5 t6=C6
    C1=S2R, C2=S3, C3=S4, C4=S2L(mirror), C5=S5, C6=S1
    """
    steps = joint_angles_to_steps(angles)

    s1 = steps.get('S1', 0)
    s2 = steps.get('S2', 0)
    s3 = steps.get('S3', 0)
    s4 = steps.get('S4', 0)
    s5 = steps.get('S5', 0)

    # syncabs t1 t2 t3 t4 t5 t6 maxSps rampSteps
    # t1=C1(S2R) t2=C2(S3) t3=C3(S4) t4=C4(S2L=S2R) t5=C5(S5) t6=C6(S1)
    max_sps   = 800
    ramp_steps = 40

    return f"syncabs {s2} {s3} {s4} {s2} {s5} {s1} {max_sps} {ramp_steps}"


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