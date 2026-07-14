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
# WORLD FRAME (meters, radians) -- to translate the origin of the arm's local 
# frame to the world frame. This is used to convert between world coordinates 
# and the arm's local coordinates.
# ---------------------------------------------------------------------------
#WORLD_BASE_POSITION = (-0.04378, 0, 0)   # from base_pos (x, y, z)
#WORLD_BASE_YAW = 0  #-math.pi/2    # from base_ori's z component (radians) z_orientation

#=== Fitted base transform (paste into config.py) ===
WORLD_BASE_POSITION = (-0.0438, 0.0000, -0.0000)
WORLD_BASE_YAW = 0.0021   # 0.12 degrees





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
L3 = 0.09447
L3_FIXED_ANGLE = -0.0990   # radians, measured via characterize_linkage.py (-5.67 deg)



# Base rotation axis -> shoulder rotation axis
H0 = 0.094   # vertical offset (m)
D0 = 0.000   # horizontal offset (m), measured ~0
"""
#=== Measured geometry (paste into config.py) ===
L1 = 0.13567   # shoulder -> elbow hinge (was 0.13500)
L2 = 0.14728   # elbow hinge -> gripper mount aux joint (was 0.14700)
L3 = 0.09490   # gripper mount aux joint -> gripper center (was 0.02900)
H0 = -0.05400   # base -> shoulder, vertical (was 0.09400)
D0 = 0.01300   # base -> shoulder, radial (was 0.00000)
"""

# ---------------------------------------------------------------------------
# ELBOW CONFIGURATION
# ---------------------------------------------------------------------------
# Which branch of the IK solution to use. This low-cost arm design almost
# always operates "elbow up" (forearm bends up/away from the ground), which
# matches the joint limits you measured in CoppeliaSim (motor3: -0.45..1.35
# rad, i.e. mostly positive). Flip to "down" only if your solved angles keep
# falling outside the physical limits after calibration.
ELBOW_CONFIG = "down"   # "up" or "down"

# ---------------------------------------------------------------------------
# JOINT LIMITS (radians) -- taken directly from your CoppeliaSim scene
# ---------------------------------------------------------------------------
JOINT_LIMITS = {
    "base":     (-3.1415, 3.1415),
    "shoulder": (-1.550, 0.356),
    "elbow":    (-0.275, 0.705),
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
    "base":     {"sign": -1, "offset": 0.0},
    "shoulder": {"sign": 1, "offset": -1.8819885},
    "elbow":    {"sign":-1, "offset": -0.0729548},
    "gripper":  {"sign": 1, "offset": 0.0},
}

# ---------------------------------------------------------------------------
# MOTION PROFILE LIMITS -- this is what fixes the jerking.
# Start conservative, then increase MAX_VEL/MAX_ACCEL until the physical arm
# just stops jerking; going higher than necessary just makes it snappier.
# ---------------------------------------------------------------------------
MAX_VEL = {      # rad/s
    "base": 20.0,
    "shoulder": 20.0,
    "elbow": 20.0,
    "gripper": 10.0,
}

MAX_ACCEL = {    # rad/s^2
    "base": 30.0,
    "shoulder": 30.0,
    "elbow": 30.0,
    "gripper": 10.0,
}

MAX_LINEAR_VEL = 20  # m/s, for cartesian moves (x,y,z)
MAX_LINEAR_ACCEL = 20  # m/s^2, for cartesian moves (x,y,z)

# Minimum move duration (s), avoids degenerate zero-length trajectories
MIN_MOVE_DURATION = 0.02

# Trajectory sampling / streaming rate to CoppeliaSim (and later, serial)
UPDATE_RATE_HZ = 30.0

# Joint order used everywhere a plain list/tuple of 4 angles is passed around
JOINT_NAMES = ("base", "shoulder", "elbow", "gripper")

# ---------------------------------------------------------------------------
# COPPELIASIM SCENE OBJECT PATHS
# ---------------------------------------------------------------------------
SIM_JOINT_PATHS = {
    "base":     "/motor1",
    "shoulder": "/motor2",
    "elbow":    "/motor3",
    "gripper":  "/motor4",
}

# Dummy object placed at the gripper center (where the cookie sits),
# parented to the end effector so it moves with the arm. Used only for
# calibration (calibrate_frame.py) to measure where the gripper ACTUALLY
# ends up vs. where a cartesian target intended it to go.
EFFECTOR_DUMMY_PATH = "/EndEffector_Dummy" 


# ---------------------------------------------------------------------------
# FORCE-FEEDBACK GRASPING
# ---------------------------------------------------------------------------
# Path to a Force Sensor object inserted into the gripper's hierarchy
# (jointObject -> forceSensor -> fingerShape), so it reads the contact
# force transmitted to the finger. Update this to your scene's actual path.
GRIPPER_FORCE_SENSOR_PATH = "/gripperForceSensor"   
 
# Force (Newtons) at which contact is considered detected and closing stops.
# START LOW and tune from the printed force log during a real grasp test --
# grasp.py prints the force at every step, so watch where it jumps from ~0
# to something real once the jaw touches the cylinder, and set this a bit
# above the "no contact" noise floor.
GRIP_DISTANCE_THRESHOLD_m = 0.0001
GRIP_FORCE_THRESHOLD_N = 0.4
 
# How far (radians) the gripper steps per increment while closing. Smaller
# = gentler / more precise contact detection, but slower and more sim steps.
GRIP_STEP_RAD = 0.050
 
# Pause (seconds) after each step before reading the force sensor, letting
# the sim settle so the reading isn't just transient contact noise.
GRIP_STEP_DELAY_S = 0.03

# timeout_s elapses (safety fallback, shouldn't normally trigger)
GRIPPER_CLOSE_TIMEOUT_S = 8.0

# ---------------------------------------------------------------------
# CONFIGURATION - Galletas
# ---------------------------------------------------------------------

# Names (scene tree paths) of your existing template objects.
# Use the object's path as shown in the scene hierarchy, e.g. "/Gear"
# or just its name if it's at the top level, e.g. "Gear".
GALLETAS_NAMES = ["/Galleta_Cuadrada", "/Galleta_Redonda", "/Galleta_Estrella"]

# How many objects to be simultanoisly 
MAX_OBJECTS = 6

# Minimum allowed distance (meters) to any existing "Galleta" object
# before a new one is allowed to spawn at a given position
MIN_SPAWN_DISTANCE = 0.27

TASA_DE_ARRIBO = 1500 # Galletas/hora
time2wait = 3600/TASA_DE_ARRIBO  # con None el tiempo de espera es aleatorio

CONVEYOR_ACCEL = 0.04  # TUNE: Aceleración máxima de tu cinta en m/s^2 