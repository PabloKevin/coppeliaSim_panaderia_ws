"""
trajectory.py
=============
Jerk-limited motion planning between two joint configurations, used to stop
the arm from snapping instantly to a new target (which is what causes the
jerking with these over-torqued servos).

Uses a quintic (5th-order) minimum-jerk polynomial time-scaling, a standard
and simple way to get a true S-curve: velocity AND acceleration are exactly
zero at the start and end of every move, and the profile stays smooth
(C2-continuous) throughout. All joints are synchronized to arrive together
by sharing a single move duration T, computed as the max duration required
by whichever joint needs to move the most relative to its own vel/accel
limits.
"""

import math
from dataclasses import dataclass
from typing import Dict, List

import config as config

# Quintic profile s(tau) = 10*tau^3 - 15*tau^4 + 6*tau^5, tau in [0, 1]
# Known closed-form peak coefficients (derived from ds/dtau, d2s/dtau2):
_PEAK_VEL_COEFF = 1.875          # max(ds/dtau) at tau=0.5
_PEAK_ACCEL_COEFF = 10.0 / math.sqrt(3.0)  # max(d2s/dtau2) at tau=0.5 -/+ 1/(2*sqrt(3))


def _quintic_s(tau: float) -> float:
    return 10 * tau ** 3 - 15 * tau ** 4 + 6 * tau ** 5


@dataclass
class TrajectorySample:
    t: float
    joint_angles: Dict[str, float]


class QuinticScurvePlanner:
    """
    Plans a jerk-limited move from one 4-joint configuration to another.

    Usage:
        planner = QuinticScurvePlanner()
        samples = planner.plan(start={"base":0,...}, goal={"base":0.3,...})
        for sample in samples:
            send_to_sim(sample.joint_angles)   # already time-ordered
    """

    def __init__(self, max_vel=config.MAX_VEL, max_accel=config.MAX_ACCEL,
                 update_rate_hz=config.UPDATE_RATE_HZ,
                 min_duration=config.MIN_MOVE_DURATION):
        self.max_vel = max_vel
        self.max_accel = max_accel
        self.dt = 1.0 / update_rate_hz
        self.min_duration = min_duration

    def _required_duration(self, joint_name: str, delta: float) -> float:
        if delta == 0.0:
            return 0.0
        v_max = self.max_vel[joint_name]
        a_max = self.max_accel[joint_name]
        t_v = _PEAK_VEL_COEFF * abs(delta) / v_max
        t_a = math.sqrt(_PEAK_ACCEL_COEFF * abs(delta) / a_max)
        return max(t_v, t_a)

    def plan(self, start: Dict[str, float], goal: Dict[str, float]) -> List[TrajectorySample]:
        """
        start, goal: dicts keyed by config.JOINT_NAMES -> angle (radians).
        Returns a time-ordered list of TrajectorySample, sampled at
        config.UPDATE_RATE_HZ, always ending exactly on the goal.
        """
        deltas = {name: goal[name] - start[name] for name in config.JOINT_NAMES}

        duration = max(
            (self._required_duration(name, deltas[name]) for name in config.JOINT_NAMES),
            default=0.0,
        )
        duration = max(duration, self.min_duration)

        n_steps = max(1, math.ceil(duration / self.dt))
        samples = []
        for i in range(n_steps + 1):
            t = min(i * self.dt, duration)
            tau = t / duration
            s = _quintic_s(tau)
            angles = {name: start[name] + deltas[name] * s for name in config.JOINT_NAMES}
            samples.append(TrajectorySample(t=t, joint_angles=angles))

        return samples

    def estimate_duration(self, start: Dict[str, float], goal: Dict[str, float]) -> float:
        """Same duration math as plan(), without generating samples. Handy for
        the routine executor to log/estimate total routine time up front."""
        deltas = {name: goal[name] - start[name] for name in config.JOINT_NAMES}
        duration = max(
            (self._required_duration(name, deltas[name]) for name in config.JOINT_NAMES),
            default=0.0,
        )
        return max(duration, self.min_duration)


class LinearCartesianPlanner:
    """
    Plans a jerk-limited move along a STRAIGHT LINE in cartesian space
    (x, y, z), instead of a straight line in joint-space. QuinticScurvePlanner
    interpolates joint angles directly, which produces a curved/paraboloid
    end-effector path even though each joint's own motion is smooth -- this
    class fixes that by interpolating position and solving IK fresh at every
    sample, so the gripper actually traces a line between start and goal.

    Uses the same quintic (minimum-jerk) time-scaling as QuinticScurvePlanner,
    but sized from a LINEAR distance (meters) and linear velocity/acceleration
    limits (config.MAX_LINEAR_VEL / MAX_LINEAR_ACCEL), not per-joint ones.
    The gripper channel is interpolated separately over the same time base,
    since it isn't part of the cartesian IK.
    """

    def __init__(self, ik, max_linear_vel=None, max_linear_accel=None,
                 update_rate_hz=config.UPDATE_RATE_HZ,
                 min_duration=config.MIN_MOVE_DURATION):
        self.ik = ik
        self.max_linear_vel = max_linear_vel if max_linear_vel is not None else config.MAX_LINEAR_VEL
        self.max_linear_accel = max_linear_accel if max_linear_accel is not None else config.MAX_LINEAR_ACCEL
        self.dt = 1.0 / update_rate_hz
        self.min_duration = min_duration

    def _required_duration(self, distance: float) -> float:
        if distance == 0.0:
            return 0.0
        t_v = _PEAK_VEL_COEFF * distance / self.max_linear_vel
        t_a = math.sqrt(_PEAK_ACCEL_COEFF * distance / self.max_linear_accel)
        return max(t_v, t_a)

    def plan(self, start_xyz, goal_xyz, start_gripper: float, goal_gripper: float) -> List[TrajectorySample]:
        """
        start_xyz, goal_xyz: (x, y, z) tuples in the SAME frame the IK
        instance expects (i.e. the arm's LOCAL frame -- convert from world
        coordinates via FrameTransform.world_to_local before calling this).
        start_gripper, goal_gripper: gripper angle (radians) to interpolate
        between over the same time base -- pass start_gripper==goal_gripper
        to hold it constant during the move.

        Returns a time-ordered list of TrajectorySample, sampled at
        config.UPDATE_RATE_HZ, always ending exactly on the goal. Raises
        kinematics.UnreachableTargetError if any point along the line
        (not just the endpoints) falls outside the reachable workspace.
        """
        dx = goal_xyz[0] - start_xyz[0]
        dy = goal_xyz[1] - start_xyz[1]
        dz = goal_xyz[2] - start_xyz[2]
        distance = math.sqrt(dx ** 2 + dy ** 2 + dz ** 2)

        duration = max(self._required_duration(distance), self.min_duration)
        n_steps = max(1, math.ceil(duration / self.dt))

        d_gripper = goal_gripper - start_gripper
        samples = []
        for i in range(n_steps + 1):
            t = min(i * self.dt, duration)
            tau = t / duration
            s = _quintic_s(tau)

            x = start_xyz[0] + dx * s
            y = start_xyz[1] + dy * s
            z = start_xyz[2] + dz * s

            angles = self.ik.solve(x, y, z, gripper_open=True)  # gripper field overwritten below
            joint_angles = dict(zip(config.JOINT_NAMES, angles.as_tuple()))
            joint_angles["gripper"] = start_gripper + d_gripper * s

            samples.append(TrajectorySample(t=t, joint_angles=joint_angles))

        return samples

    def estimate_duration(self, start_xyz, goal_xyz) -> float:
        dx = goal_xyz[0] - start_xyz[0]
        dy = goal_xyz[1] - start_xyz[1]
        dz = goal_xyz[2] - start_xyz[2]
        distance = math.sqrt(dx ** 2 + dy ** 2 + dz ** 2)
        return max(self._required_duration(distance), self.min_duration)