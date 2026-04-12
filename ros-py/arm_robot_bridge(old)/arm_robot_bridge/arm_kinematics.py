import math
import numpy as np

# ─────────────────────────────────────────
# Unit conversion
# ─────────────────────────────────────────
INCH = 0.0254  # metres per inch

# ─────────────────────────────────────────
# Link lengths (metres)
# ─────────────────────────────────────────
L1 = 6.335 * INCH   # S1 to S2 (vertical, always)
L2 = 10.0  * INCH   # S2 to S3
L3 = 8.0   * INCH   # S3 to S4
L4 = 3.209 * INCH   # S4 to S5
L5 = 1.143 * INCH   # S5 to connector end
LARM = L4 + L5      # combined wrist length

# ─────────────────────────────────────────
# Tool tip offsets in connector frame
# z = along connector axis (outward)
# y = upward along connector
# x = right of connector
# ─────────────────────────────────────────
TOOLS = {
    "probe":   np.array([0.0,          0.0,          5.350 * INCH]),
    "saw":     np.array([0.0,          4.705 * INCH, 2.197 * INCH]),
    "vacuum":  np.array([0.0,          0.0,          2.635 * INCH]),
    "none":    np.array([0.0,          0.0,          0.0         ]),
}

# ─────────────────────────────────────────
# Joint zero offsets
# These convert from "math convention" angles
# to your physical joint zero convention.
#
# S2: math 0 = horizontal, physical 0 = straight up
#     so physical = math - 90°
# S3: math 0 = straight, physical 0 = straight up
#     negative physical lowers toward +X when S2=0
# S4: same convention as S3
# ─────────────────────────────────────────
S2_ZERO_OFFSET = math.radians(90)   # math measures from horizontal, yours from vertical
S3_ZERO_OFFSET = 0.0                # needs calibration
S4_ZERO_OFFSET = 0.0                # needs calibration

# ─────────────────────────────────────────
# Joint limits (radians)
# ─────────────────────────────────────────
LIMITS = {
    'S1': (math.radians(-129), math.radians(156)),
    'S2': (math.radians(-94.16), math.radians(86)),
    'S3': (math.radians(-113), math.radians(79.6)),
    'S4': (math.radians(-83),  math.radians(79)),
    'S5': (math.radians(-180), math.radians(180)),
}

def check_limits(angles_dict):
    """Check all joint angles are within limits. Returns (ok, violations)."""
    violations = []
    for joint, angle in angles_dict.items():
        lo, hi = LIMITS[joint]
        if angle < lo or angle > hi:
            violations.append(
                f"{joint}: {math.degrees(angle):.1f}° out of range "
                f"[{math.degrees(lo):.1f}°, {math.degrees(hi):.1f}°]"
            )
    return len(violations) == 0, violations

def tool_offset_world(tool_name, pointing_down=True):
    """
    Transform tool tip offset from connector frame to world frame.
    When pointing_down=True, connector z-axis points in world -Z direction.
    Returns world-frame offset (dx, dy, dz).
    """
    offset_local = TOOLS.get(tool_name, TOOLS["none"])

    if pointing_down:
        # Connector frame: z→ world -Z, y→ world +Y, x→ world +X
        # (arm pointing straight down, no S5 twist)
        dx =  offset_local[0]
        dy =  offset_local[1]
        dz = -offset_local[2]   # connector z points down → world -z
    else:
        # General case would need full rotation matrix — extend later
        raise NotImplementedError("Non-vertical tool transform not yet implemented")

    return dx, dy, dz

def inverse_kinematics(x, y, z, s5_angle=0.0, tool="none"):
    """
    Compute joint angles to place the tool tip at world point (x, y, z)
    with the arm pointing straight down.

    Parameters:
        x, y, z   : target world coordinates (metres), relative to S1 base
        s5_angle  : desired wrist twist angle (radians)
        tool      : tool name for tip offset ("probe", "saw", "vacuum", "none")

    Returns:
        dict of joint angles in radians (physical convention), or None if unreachable
    """

    # ── Step 1: subtract tool tip offset to find connector-end target ──
    dx, dy, dz = tool_offset_world(tool, pointing_down=True)
    cx = x - dx
    cy = y - dy
    cz = z - dz

    # ── Step 2: subtract wrist (L4+L5) — arm points straight down
    # so wrist hangs straight down from S4
    wx = cx
    wy = cy
    wz = cz + LARM   # S4 is LARM above the connector end when pointing down

    # ── Step 3: S1 base rotation ──
    theta1 = math.atan2(wy, wx)

    # ── Step 4: project into the vertical plane of the arm ──
    r = math.sqrt(wx**2 + wy**2)   # horizontal distance from base
    h = wz - L1                     # height above S2

    # Distance from S2 to S4
    d = math.sqrt(r**2 + h**2)

    # Check reachability
    if d > L2 + L3:
        print(f"Target unreachable: distance {d*100:.1f}cm > max reach {(L2+L3)*100:.1f}cm")
        return None
    if d < abs(L2 - L3):
        print(f"Target unreachable: distance {d*100:.1f}cm < min reach {abs(L2-L3)*100:.1f}cm")
        return None

    # ── Step 5: solve elbow (S3) via law of cosines ──
    cos_s3 = (d**2 - L2**2 - L3**2) / (2 * L2 * L3)
    cos_s3 = max(-1.0, min(1.0, cos_s3))   # clamp for floating point safety
    s3_math = math.acos(cos_s3)             # elbow-up solution

    # ── Step 6: solve shoulder (S2) ──
    alpha = math.atan2(h, r)
    beta  = math.acos(
        max(-1.0, min(1.0, (L2**2 + d**2 - L3**2) / (2 * L2 * d)))
    )
    s2_math = alpha + beta

    # ── Step 7: wrist pitch (S4) to keep arm pointing straight down ──
    # When arm points straight down, the total joint angle sum must
    # result in the wrist pointing at -90° from horizontal (i.e. straight down)
    # s2_math is measured from horizontal, so:
    # s4_math = -(pi/2) - s2_math - s3_math  (need to point at -90° total)
    s4_math = -(math.pi / 2) - s2_math - s3_math

    # ── Step 8: convert to physical joint conventions ──
    # S2: physical 0 = vertical up = math 90°
    s2_physical = s2_math - S2_ZERO_OFFSET
    # S3: negative physical lowers toward +X (same sign as math convention here)
    s3_physical = -s3_math   # elbow folds the other way in your convention
    # S4: same
    s4_physical = -s4_math - S2_ZERO_OFFSET

    angles = {
        'S1': theta1,
        'S2': s2_physical,
        'S3': s3_physical,
        'S4': s4_physical,
        'S5': s5_angle,
    }

    # ── Step 9: check joint limits ──
    ok, violations = check_limits(angles)
    if not ok:
        print("Joint limit violations:")
        for v in violations:
            print(f"  {v}")
        return None

    return angles

def angles_to_degrees(angles):
    """Helper to print angles in degrees."""
    return {k: round(math.degrees(v), 2) for k, v in angles.items()}

# ─────────────────────────────────────────
# Docking station positions (metres)
# ─────────────────────────────────────────
DOCKING_STATIONS = {
    "probe":   (9.515 * INCH, 4.434 * INCH,  0.0),
    "saw":     (10.5  * INCH, 0.0,           0.0),
    "vacuum":  (9.515 * INCH, -4.434 * INCH, 0.0),
}

if __name__ == "__main__":
    # Test: point at the saw docking station with no tool attached
    target = DOCKING_STATIONS["saw"]
    print(f"Target: {target}")
    
    result = inverse_kinematics(*target, s5_angle=0.0, tool="none")
    
    if result:
        print("Joint angles (degrees):")
        for joint, deg in angles_to_degrees(result).items():
            print(f"  {joint}: {deg}°")
    else:
        print("No solution found")

# import math
# import numpy as np

# ### THIS IS PURE CHAT STILL ###

# # Convert inches → meters (ROS standard)
# INCH = 0.0254

# TOOLS = {
#     "probe": np.array([0.0, 0.0, 5.350 * INCH]),
#     "saw": np.array([0.0, 4.705 * INCH, 2.197 * INCH]),
#     "vacuum": np.aray([0.0, 0.0, 2.635 * INCH]),
#     "none": np.array([0.0, 0.0, 0.0]),
# }

# L1 = 6.335 * INCH
# L2 = 10.0 * INCH
# L3 = 8.0 * INCH
# L4 = 3.209 * INCH
# L5 = 1.143 * INCH

# def inverse_kinematics(x, y, z):
#     """
#     Returns joint angles (theta1–theta4)
#     """

#     # Base rotation
#     theta1 = math.atan2(y, x)

#     # Project into planar arm
#     r = math.sqrt(x**2 + y**2)
#     z_offset = z - L1

#     # Distance from shoulder joint
#     d = math.sqrt(r**2 + z_offset**2)

#     # Law of cosines for elbow
#     cos_theta3 = (d**2 - L2**2 - L3**2) / (2 * L2 * L3)
#     theta3 = math.acos(cos_theta3)

#     # Shoulder angle
#     alpha = math.atan2(z_offset, r)
#     beta = math.acos((L2**2 + d**2 - L3**2) / (2 * L2 * d))
#     theta2 = alpha + beta

#     # Wrist pitch (to keep vertical downward)
#     theta4 = -(theta2 + theta3)

#     return theta1, theta2, theta3, theta4

# def inverse_kinematics_with_tool(x, y, z, tool):
#     dx, dy, dz = TOOLS[tool]

#     return inverse_kinematics(
#         x - dx,
#         y - dy,
#         z - dz
#     )