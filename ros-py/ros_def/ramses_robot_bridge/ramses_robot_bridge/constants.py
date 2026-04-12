"""
constants.py
============
Arm geometry and conversion factors for the ME424 5-DOF robotic arm.

Tune these values to match your actual hardware:
  - STEPS_PER_REV  : full steps per motor revolution (usually 200)
  - MICROSTEPS     : microstepping divisor set on your driver (1, 2, 4, 8, 16 ...)
  - GEAR_RATIO     : output turns per motor turn (1.0 if direct drive)

steps_per_output_rev = STEPS_PER_REV * MICROSTEPS * GEAR_RATIO
radians_per_step     = 2π / steps_per_output_rev

Joint names must exactly match those in your URDF / RViz2 robot model.
"""

import math

# ------------------------------------------------------------------
# Per-stage drive configuration
# Edit these to match your real hardware setup.
# ------------------------------------------------------------------

# Each entry: (steps_per_motor_rev, microsteps, gear_ratio)
# Index matches controller numbering:
#   0 = C1 / STAGE2_RIGHT (base lift)
#   1 = C2 / STAGE3       (arm segment)
#   2 = C3 / STAGE4       (arm segment)
#   3 = C4 / STAGE2_LEFT  (base lift, mirrored — same joint as C1)
#   4 = C5 / STAGE5       (wrist)
#   5 = C6 / STAGE1       (turntable)

_DRIVE = [
    # (steps/rev, microsteps, gear_ratio)
    (200, 16, 1.0),   # C1 / S2 right
    (200, 16, 1.0),   # C2 / S3
    (200, 16, 1.0),   # C3 / S4
    (200, 16, 1.0),   # C4 / S2 left  (mirrored, same joint as C1)
    (200, 16, 1.0),   # C5 / S5 wrist
    (200, 16, 1.0),   # C6 / S1 turntable
]

def _rad_per_step(idx: int) -> float:
    spr, ms, gr = _DRIVE[idx]
    steps_per_output_rev = spr * ms * gr
    return (2.0 * math.pi) / steps_per_output_rev


# Pre-computed radians-per-step for each controller index
RAD_PER_STEP = [_rad_per_step(i) for i in range(6)]

# ------------------------------------------------------------------
# Joint names — must match your URDF exactly
# ------------------------------------------------------------------
# We publish 5 joints (S2 left/right share one joint in the model).
JOINT_NAMES = [
    'joint_turntable',   # C6 / S1
    'joint_lift',        # C1+C4 / S2  (paired, use C1 value)
    'joint_arm1',        # C2 / S3
    'joint_arm2',        # C3 / S4
    'joint_wrist',       # C5 / S5
]

# ------------------------------------------------------------------
# Serial port defaults (can be overridden by ROS2 parameters)
# ------------------------------------------------------------------
DEFAULT_PORT     = '/dev/ttyUSB0'
DEFAULT_BAUDRATE = 115200

# ------------------------------------------------------------------
# Timeouts
# ------------------------------------------------------------------
# How long to wait for ACK after sending a command (seconds).
ACK_TIMEOUT_S = 2.0

# How long to wait for DONE/ERR after ACK (seconds).
# Set high enough for your longest expected motion.
DONE_TIMEOUT_S = 120.0

# Serial readline timeout (seconds) — keep short so the reader
# thread stays responsive.
SERIAL_READ_TIMEOUT_S = 0.1

# ------------------------------------------------------------------
# Commands treated as fire-and-forget via the /arm/command topic
# (these never wait for DONE — they return immediately after ACK)
# ------------------------------------------------------------------
INSTANT_COMMANDS = {
    'estop', 'estop clear', 'estop status',
    'mag on', 'mag off',
    'vac on', 'vac off',
    'saw on', 'saw off',
    'alloff', 'mstatus',
    'debug on', 'debug off',
    'qclear', 'qstop', 'qstatus', 'qlist',
    'where', 'limits', 'help',
}
