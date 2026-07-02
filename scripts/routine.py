"""
routine.py
==========
Defines a "routine" as an ordered list of cartesian waypoints (with an
associated gripper state and optional post-move wait), loads it from a JSON
file, and executes it as a simple state machine: for each waypoint ->
solve IK -> plan a jerk-limited trajectory from the current pose -> stream
it to the active interface (sim now, serial later) -> wait if requested ->
move to next waypoint.

This is the layer you'll touch most often day-to-day: editing the JSON file
changes the whole pick-and-place sequence without touching any code.
"""

import json
import time
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional

from kinematics import InverseKinematics, JointCalibration
from trajectory import QuinticScurvePlanner
import config as config


@dataclass
class Waypoint:
    label: str
    x: float
    y: float
    z: float
    gripper_open: bool
    wait_after_s: float = 0.0


class RoutineLoader:
    """Loads a routine from a JSON file. Expected format:

    {
      "waypoints": [
        {"label": "above_cookie", "x": 0.15, "y": 0.05, "z": 0.10,
         "gripper_open": true, "wait_after_s": 0.3},
        ...
      ]
    }

    Cartesian coordinates are in meters, in the arm's base frame (x forward,
    y sideways, z up from the table / base rotation axis level).
    """

    @staticmethod
    def load(path: str) -> List[Waypoint]:
        data = json.loads(Path(path).read_text())
        waypoints = []
        for wp in data["waypoints"]:
            waypoints.append(Waypoint(
                label=wp.get("label", ""),
                x=wp["x"], y=wp["y"], z=wp["z"],
                gripper_open=wp["gripper_open"],
                wait_after_s=wp.get("wait_after_s", 0.0),
            ))
        return waypoints


class RoutineExecutor:
    """
    Wires together IK, trajectory planning, calibration, and an interface
    (anything exposing get_current_joint_angles() / stream_trajectory()) to
    run a full routine.

    `interface` is duck-typed on purpose: sim_interface.CoppeliaSimController
    today, a future serial_interface.SerialController tomorrow -- both just
    need get_current_joint_angles() and stream_trajectory().
    """

    def __init__(self, interface, ik: Optional[InverseKinematics] = None,
                 planner: Optional[QuinticScurvePlanner] = None,
                 calibration: Optional[JointCalibration] = None,
                 verbose: bool = True):
        self.interface = interface
        self.ik = ik or InverseKinematics()
        self.planner = planner or QuinticScurvePlanner()
        self.calibration = calibration or JointCalibration()
        self.verbose = verbose

    def _log(self, msg: str):
        if self.verbose:
            print(msg)

    def run(self, waypoints: List[Waypoint]):
        current_angles = self.interface.get_current_joint_angles()

        for wp in waypoints:
            target_angles = self.ik.solve(wp.x, wp.y, wp.z, wp.gripper_open)
            target_dict = dict(zip(config.JOINT_NAMES, target_angles.as_tuple()))

            est_duration = self.planner.estimate_duration(current_angles, target_dict)
            self._log(f"-> {wp.label or '(unlabeled)'}: "
                      f"({wp.x:.3f}, {wp.y:.3f}, {wp.z:.3f}) "
                      f"gripper={'open' if wp.gripper_open else 'closed'} "
                      f"[~{est_duration:.2f}s]")

            samples = self.planner.plan(current_angles, target_dict)
            self.interface.stream_trajectory(samples, self.calibration)

            if wp.wait_after_s > 0:
                time.sleep(wp.wait_after_s)

            current_angles = target_dict

        self._log("Routine complete.")
