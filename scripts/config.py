"""
config.py
=========
Single source of truth for every tunable constant used by the arm controller:
geometry, joint limits, motion-profile limits (velocity/acceleration), and
per-joint calibration (sign + offset) needed to map our internal "absolute
angle" convention onto the real servo / CoppeliaSim joint values.

Keep ALL magic numbers here. Nothing else in the project should hardcode
a link length, a limit, or a calibration value.
"""

import math

# ---------------------------------------------------------------------------
# ARM GEOMETRY (meters) -- measured on the physical robot
# ---------------------------------------------------------------------------
# L1: shoulder rotation axis -> elbow hinge axis (upper arm)
L1 = 0.135

# L2: elbow hinge axis -> gripper mounting aux joint (forearm, via parallel
#     linkage bracket)
L2 = 0.147

# L3: gripper mounting aux joint -> gripper center (where the cookie sits).
#     ASSUMPTION: colinear with L2, i.e. it just extends the forearm bracket
#     in the same direction. If your gripper mount sits at a fixed angular
#     offset from the forearm bracket instead, add that offset here and use
#     it in kinematics.py when computing the effective forearm angle.
L3 = 0.029

# Effective forearm length used directly in the 2-link planar IK.
L2_EFF = L2 + L3

# Base rotation axis -> shoulder rotation axis
H0 = 0.094   # vertical offset (m)
D0 = 0.000   # horizontal offset (m), measured ~0

# ---------------------------------------------------------------------------
# ELBOW CONFIGURATION
# ---------------------------------------------------------------------------
# Which branch of the IK solution to use. This low-cost arm design almost
# always operates "elbow up" (forearm bends up/away from the ground), which
# matches the joint limits you measured in CoppeliaSim (motor3: -0.45..1.35
# rad, i.e. mostly positive). Flip to "down" only if your solved angles keep
# falling outside the physical limits after calibration.
ELBOW_CONFIG = "up"   # "up" or "down"

# ---------------------------------------------------------------------------
# JOINT LIMITS (radians) -- taken directly from your CoppeliaSim scene
# ---------------------------------------------------------------------------
JOINT_LIMITS = {
    "base":     (-3.1415, 3.1415),
    "shoulder": (-1.550, 0.950),
    "elbow":    (-0.450, 1.350),
    "gripper":  (0.000, 1.450),
}

# ---------------------------------------------------------------------------
# GRIPPER
# ---------------------------------------------------------------------------
# Treated as a simple boolean actuator (open/closed), not full-range.
# 0.8 rad chosen per your note as "open enough" without over-extending.
GRIPPER_CLOSED = 0.0
GRIPPER_OPEN = 0.8

# ---------------------------------------------------------------------------
# CALIBRATION: maps our internal "absolute angle from horizontal" convention
# to the actual joint command value (sim radians now, servo degrees later).
#
#   commanded_value = SIGN * internal_angle + OFFSET
#
# We don't yet know the real-world zero references, so these start as
# identity (sign=1, offset=0). Tune them once you can compare: command a
# known internal angle (e.g. theta1 = 0 for "shoulder horizontal") and
# observe where the joint actually goes, then adjust OFFSET; flip SIGN if
# increasing the internal angle moves the joint the wrong way.
# ---------------------------------------------------------------------------
CALIBRATION = {
    "base":     {"sign": 1, "offset": 0.0},
    "shoulder": {"sign": 1, "offset": 0.0},
    "elbow":    {"sign": 1, "offset": 0.0},
    "gripper":  {"sign": 1, "offset": 0.0},
}

# ---------------------------------------------------------------------------
# MOTION PROFILE LIMITS -- this is what fixes the jerking.
# Start conservative, then increase MAX_VEL/MAX_ACCEL until the physical arm
# just stops jerking; going higher than necessary just makes it snappier.
# ---------------------------------------------------------------------------
MAX_VEL = {      # rad/s
    "base": 1.0,
    "shoulder": 0.8,
    "elbow": 0.8,
    "gripper": 2.0,
}

MAX_ACCEL = {    # rad/s^2
    "base": 1.5,
    "shoulder": 1.2,
    "elbow": 1.2,
    "gripper": 4.0,
}

# Minimum move duration (s), avoids degenerate zero-length trajectories
MIN_MOVE_DURATION = 0.15

# Trajectory sampling / streaming rate to CoppeliaSim (and later, serial)
UPDATE_RATE_HZ = 50.0

# Joint order used everywhere a plain list/tuple of 4 angles is passed around
JOINT_NAMES = ("base", "shoulder", "elbow", "gripper")

# ---------------------------------------------------------------------------
# COPPELIASIM SCENE OBJECT PATHS
# ---------------------------------------------------------------------------
SIM_JOINT_PATHS = {
    "base":     "/EZZYbotARM_Mk2/motor1",
    "shoulder": "/EZZYbotARM_Mk2/motor2",
    "elbow":    "/EZZYbotARM_Mk2/motor3",
    "gripper":  "/EZZYbotARM_Mk2/motor4",
}
