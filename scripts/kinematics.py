"""
kinematics.py
=============
Analytical inverse kinematics for the 4-DOF arm (base yaw + 2-link planar
parallelogram arm + gripper).

Key mechanical insight (confirmed on the physical arm): because the "elbow"
servo drives a parallel rod from near the base rather than being mounted at
the shoulder joint, the elbow servo commands the forearm's ABSOLUTE angle
(measured from the same reference as the shoulder, e.g. horizontal) rather
than an angle relative to the upper arm. This decouples shoulder and elbow
mechanically and simplifies the IK: standard 2-link law-of-cosines IK gives
theta1 (shoulder, absolute) and theta2_rel (elbow bend relative to the upper
arm); we then convert theta2_rel to theta2_abs = theta1 + theta2_rel before
it is used as the elbow command.
"""

import math
from dataclasses import dataclass

import config


class FrameTransform:
    """
    Converts a cartesian point given in CoppeliaSim's WORLD frame into the
    robot-local frame that InverseKinematics expects (origin at the base
    joint's rotation axis, x = direction the arm points at theta_base=0).

    Fill config.WORLD_BASE_POSITION and config.WORLD_BASE_YAW using
    diagnose_frame.py, then every waypoint you write can just be the
    coordinates you read directly off the object in CoppeliaSim.
    """

    def __init__(self, base_position=config.WORLD_BASE_POSITION,
                 base_yaw=config.WORLD_BASE_YAW):
        self.base_position = base_position
        self.base_yaw = base_yaw

    def world_to_local(self, x: float, y: float, z: float):
        # Translate so the base joint is the origin
        dx = x - self.base_position[0]
        dy = y - self.base_position[1]
        dz = z - self.base_position[2]

        # Un-rotate by the base's yaw so "local x" matches theta_base = 0
        cos_a = math.cos(-self.base_yaw)
        sin_a = math.sin(-self.base_yaw)
        lx = dx * cos_a - dy * sin_a
        ly = dx * sin_a + dy * cos_a

        return lx, ly, dz

    def local_to_world(self, x: float, y: float, z: float):
        """Inverse of world_to_local -- mainly useful for sanity checks."""
        cos_a = math.cos(self.base_yaw)
        sin_a = math.sin(self.base_yaw)
        dx = x * cos_a - y * sin_a
        dy = x * sin_a + y * cos_a

        wx = dx + self.base_position[0]
        wy = dy + self.base_position[1]
        wz = z + self.base_position[2]
        return wx, wy, wz


class UnreachableTargetError(Exception):
    """Raised when a cartesian target is outside the arm's reachable workspace."""
    pass


@dataclass
class JointAngles:
    base: float
    shoulder: float
    elbow: float
    gripper: float

    def as_tuple(self):
        return (self.base, self.shoulder, self.elbow, self.gripper)


class InverseKinematics:
    """
    Converts cartesian targets (in the arm's base frame, meters, z up from
    the table / base-rotation-axis level) into the 4 absolute joint angles
    (radians) used internally by the rest of the pipeline.

    MECHANISM (confirmed empirically via characterize_linkage.py): L1 is
    driven directly by the shoulder servo (absolute angle theta1). L2's
    absolute angle (theta2_abs) is set directly by the elbow servo via the
    parallel-rod linkage, independent of the shoulder. L3 is a SEPARATE
    parallelogram output link with a FIXED absolute angle in the world
    frame (L3_FIXED_ANGLE) -- it does not rotate with L2 at all. So L3
    contributes a constant offset vector to the end-effector position; the
    2-link IK below is solved for L1/L2 placing the auxMotor2 point, and
    the constant L3 offset is added/subtracted around that solve.

    NOTE: the angles returned here are in our own "absolute angle from
    horizontal" convention. They are NOT yet the values to command to the
    simulator or a real servo -- that mapping (sign/offset) lives in
    config.CALIBRATION and is applied by JointCalibration.to_command().
    """

    def __init__(self, l1=config.L1, l2=config.L2, l3=config.L3,
                 l3_angle=config.L3_FIXED_ANGLE,
                 h0=config.H0, d0=config.D0,
                 elbow_config=config.ELBOW_CONFIG):
        self.l1 = l1
        self.l2 = l2
        self.l3 = l3
        self.l3_angle = l3_angle
        self.h0 = h0
        self.d0 = d0
        self.elbow_config = elbow_config

        # Constant offset contributed by L3, since its absolute angle never
        # changes with arm pose.
        self._l3_offset_r = self.l3 * math.cos(self.l3_angle)
        self._l3_offset_z = self.l3 * math.sin(self.l3_angle)

    def solve(self, x: float, y: float, z: float, gripper_open: bool) -> JointAngles:
        """
        x, y, z: target position of the gripper center, in the arm's base
                 frame (meters). z=0 is table level / base rotation axis.
        gripper_open: True -> gripper target = config.GRIPPER_OPEN
                      False -> gripper target = config.GRIPPER_CLOSED

        Returns a JointAngles instance. Raises UnreachableTargetError if the
        target is outside the reachable annulus of the 2-link (L1, L2) arm,
        once the fixed L3 offset has been accounted for.
        """
        theta_base = math.atan2(y, x)

        # Project into the arm's vertical plane (distance from the base's
        # rotation axis, minus the small horizontal shoulder offset).
        r = math.hypot(x, y) - self.d0
        z_rel = z - self.h0

        # Subtract L3's constant contribution: we now need to place
        # auxMotor2 (end of L2), not the gripper center directly.
        r2 = r - self._l3_offset_r
        z2 = z_rel - self._l3_offset_z
        d = math.hypot(r2, z2)

        d_min = abs(self.l1 - self.l2)
        d_max = self.l1 + self.l2
        if d < d_min or d > d_max:
            raise UnreachableTargetError(
                f"Target (x={x:.3f}, y={y:.3f}, z={z:.3f}) requires "
                f"auxMotor2 at planar distance {d:.4f} m from the shoulder "
                f"axis (after removing the fixed L3 offset), outside the "
                f"reachable range [{d_min:.4f}, {d_max:.4f}] m."
            )

        cos_theta2_rel = (d ** 2 - self.l1 ** 2 - self.l2 ** 2) / (2 * self.l1 * self.l2)
        cos_theta2_rel = max(-1.0, min(1.0, cos_theta2_rel))  # guard fp error at workspace edge

        theta2_rel_mag = math.acos(cos_theta2_rel)
        theta2_rel = theta2_rel_mag if self.elbow_config == "up" else -theta2_rel_mag

        theta1 = math.atan2(z2, r2) - math.atan2(
            self.l2 * math.sin(theta2_rel),
            self.l1 + self.l2 * math.cos(theta2_rel),
        )

        theta2_abs = theta1 + theta2_rel

        gripper_angle = config.GRIPPER_OPEN if gripper_open else config.GRIPPER_CLOSED

        return JointAngles(base=theta_base, shoulder=theta1, elbow=theta2_abs,
                            gripper=gripper_angle)

    def forward(self, angles: JointAngles):
        """
        Forward kinematics, mainly useful for sanity-checking calibration
        and for unit tests. Returns (x, y, z) of the gripper center.
        """
        theta1 = angles.shoulder
        theta2_abs = angles.elbow

        r = (self.l1 * math.cos(theta1) + self.l2 * math.cos(theta2_abs)
             + self._l3_offset_r + self.d0)
        z = (self.l1 * math.sin(theta1) + self.l2 * math.sin(theta2_abs)
             + self._l3_offset_z + self.h0)

        x = r * math.cos(angles.base)
        y = r * math.sin(angles.base)
        return x, y, z


class JointCalibration:
    """
    Applies config.CALIBRATION (per-joint sign + offset) to convert internal
    absolute angles into the value actually sent to the simulator (and,
    later, the raw servo command). Kept as its own small class so the same
    calibration table can be reused by the firmware-facing code without
    duplicating the sign/offset logic.
    """

    def __init__(self, calibration=config.CALIBRATION, limits=config.JOINT_LIMITS):
        self.calibration = calibration
        self.limits = limits

    def to_command(self, joint_name: str, internal_angle: float) -> float:
        cal = self.calibration[joint_name]
        value = cal["sign"] * internal_angle + cal["offset"]

        lo, hi = self.limits[joint_name]
        if value < lo or value > hi:
            raise ValueError(
                f"Calibrated command for '{joint_name}' = {value:.4f} rad is "
                f"outside joint limits [{lo:.4f}, {hi:.4f}]. Check calibration "
                f"offsets in config.py or the requested cartesian target."
            )
        return value

    def from_command(self, joint_name: str, raw_value: float) -> float:
        """
        Inverse of to_command(): converts a raw sim/servo value back into
        our internal absolute-angle convention. Needed whenever we read the
        arm's CURRENT pose (e.g. sim.getJointPosition) and want to use it as
        a trajectory start point alongside IK-computed (internal-space)
        targets -- mixing raw and internal angles directly is a bug, since
        they're only the same when calibration is identity (sign=1, offset=0).
        """
        cal = self.calibration[joint_name]
        return (raw_value - cal["offset"]) / cal["sign"]

    def commands_to_angles(self, raw_commands: dict) -> dict:
        return {
            name: self.from_command(name, value)
            for name, value in raw_commands.items()
        }

    def angles_to_commands(self, angles: JointAngles) -> dict:
        return {
            name: self.to_command(name, value)
            for name, value in zip(config.JOINT_NAMES, angles.as_tuple())
        }