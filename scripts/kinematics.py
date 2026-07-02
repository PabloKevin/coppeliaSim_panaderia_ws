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

import config as config


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

    NOTE: the angles returned here are in our own "absolute angle from
    horizontal" convention. They are NOT yet the values to command to the
    simulator or a real servo -- that mapping (sign/offset) lives in
    config.CALIBRATION and is applied by JointCalibration.to_command().
    """

    def __init__(self, l1=config.L1, l2_eff=config.L2_EFF,
                 h0=config.H0, d0=config.D0,
                 elbow_config=config.ELBOW_CONFIG):
        self.l1 = l1
        self.l2 = l2_eff
        self.h0 = h0
        self.d0 = d0
        self.elbow_config = elbow_config

    def solve(self, x: float, y: float, z: float, gripper_open: bool) -> JointAngles:
        """
        x, y, z: target position of the gripper center, in the arm's base
                 frame (meters). z=0 is table level / base rotation axis.
        gripper_open: True -> gripper target = config.GRIPPER_OPEN
                      False -> gripper target = config.GRIPPER_CLOSED

        Returns a JointAngles instance. Raises UnreachableTargetError if the
        target is outside the reachable annulus of the 2-link arm.
        """
        theta_base = math.atan2(y, x)

        # Project into the arm's vertical plane (distance from the base's
        # rotation axis, minus the small horizontal shoulder offset).
        r = math.hypot(x, y) - self.d0
        z_rel = z - self.h0
        d = math.hypot(r, z_rel)

        d_min = abs(self.l1 - self.l2)
        d_max = self.l1 + self.l2
        if d < d_min or d > d_max:
            raise UnreachableTargetError(
                f"Target (x={x:.3f}, y={y:.3f}, z={z:.3f}) is at planar "
                f"distance {d:.4f} m from the shoulder axis, outside the "
                f"reachable range [{d_min:.4f}, {d_max:.4f}] m."
            )

        cos_theta2_rel = (d ** 2 - self.l1 ** 2 - self.l2 ** 2) / (2 * self.l1 * self.l2)
        cos_theta2_rel = max(-1.0, min(1.0, cos_theta2_rel))  # guard fp error at workspace edge

        theta2_rel_mag = math.acos(cos_theta2_rel)
        theta2_rel = theta2_rel_mag if self.elbow_config == "up" else -theta2_rel_mag

        theta1 = math.atan2(z_rel, r) - math.atan2(
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

        r = self.l1 * math.cos(theta1) + self.l2 * math.cos(theta2_abs) + self.d0
        z = self.l1 * math.sin(theta1) + self.l2 * math.sin(theta2_abs) + self.h0

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

    def angles_to_commands(self, angles: JointAngles) -> dict:
        return {
            name: self.to_command(name, value)
            for name, value in zip(config.JOINT_NAMES, angles.as_tuple())
        }
