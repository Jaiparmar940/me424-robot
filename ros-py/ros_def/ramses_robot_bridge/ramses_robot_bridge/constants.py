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
# ------------------------------------------------------------------
#
# microsteps_per_rev = full_steps_per_rev × microstep_divisor
# All stages run at 16x microstepping.
#
# Stage | Full steps/rev | ×16  | microsteps/rev
# ------+----------------+------+---------------
# S1    |   1040         | ×16  |  16640   (turntable,  C6)
# S2    |   3600         | ×16  |  57600   (base lift,  C1+C4)
# S3    |   2400         | ×16  |  38400   (arm seg 1,  C2)
# S4    |   2100         | ×16  |  33600   (arm seg 2,  C3)
# S5    |    200         | ×16  |   3200   (wrist,      C5)
#
# Index matches controller numbering (0-based):
#   0 = C1 / STAGE2_RIGHT (base lift)
#   1 = C2 / STAGE3       (arm segment 1)
#   2 = C3 / STAGE4       (arm segment 2)
#   3 = C4 / STAGE2_LEFT  (base lift, mirrored — same joint as C1)
#   4 = C5 / STAGE5       (wrist)
#   5 = C6 / STAGE1       (turntable)

# Stored as microsteps per full output revolution (already includes gear ratio).
_MICROSTEPS_PER_REV = [
    28800,   # C1 / S2 right  (base lift)
    19200,   # C2 / S3        (arm segment 1)
    16800,   # C3 / S4        (arm segment 2)
    28800,   # C4 / S2 left   (base lift, mirrors C1)
     1600,   # C5 / S5        (wrist)
    8320,   # C6 / S1        (turntable)
]

def _rad_per_step(idx: int) -> float:
    return (2.0 * math.pi) / _MICROSTEPS_PER_REV[idx]


# Pre-computed radians-per-step for each controller index
RAD_PER_STEP = [_rad_per_step(i) for i in range(6)]

# ------------------------------------------------------------------
# Joint names — must match the URDF exactly
# ------------------------------------------------------------------
# Order matches the controller-to-joint mapping used in
# _publish_joint_states (controller_for_joint list).
# JOINT_NAMES = [
#     's1_turntable',   # C6 / S1  — turntable
#     's2_shoulder',    # C1 / S2  — base lift (paired C1+C4, use C1)
#     's3_elbow',       # C2 / S3  — arm segment 1
#     's4_wrist',       # C3 / S4  — arm segment 2
#     's5_wrist_twist', # C5 / S5  — wrist twist
# ]
JOINT_NAMES = [
    's5_wrist_twist',
    's4_wrist',
    's3_elbow',
    's2_shoulder',
    's1_turntable',
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