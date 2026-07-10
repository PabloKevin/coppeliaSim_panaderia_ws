"""
routine.py
==========
Defines a "routine" as an ordered list of cartesian waypoints (with an
associated gripper state and optional post-move wait), loads it from a JSON
file, and executes it as a simple state machine.

Updated to support Linear Cartesian planning with a continuous S-curve velocity profile.
"""

import json
import time
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional

from kinematics import InverseKinematics, JointCalibration, FrameTransform, JointAngles
from trajectory import LinearCartesianPlanner
from grasp import AdaptiveGripper
import config


@dataclass
class Waypoint:
    label: str
    x: float
    y: float
    z: float
    gripper_open: bool   
    wait_after_s: float = 0.0
    adaptive_grasp: bool = False   


class RoutineLoader:
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
                adaptive_grasp=wp.get("adaptive_grasp", False),
            ))
        return waypoints


class RoutineExecutor:
    def __init__(self, interface, ik: Optional[InverseKinematics] = None,
                 planner: Optional[LinearCartesianPlanner] = None,
                 calibration: Optional[JointCalibration] = None,
                 frame_transform: Optional[FrameTransform] = None,
                 adaptive_gripper: Optional[AdaptiveGripper] = None,
                 verbose: bool = True):
        self.interface = interface
        self.ik = ik or InverseKinematics()
        # Explicitly default to the LinearCartesianPlanner for straight-line motions
        self.planner = planner or LinearCartesianPlanner(self.ik) 
        self.calibration = calibration or JointCalibration()
        self.frame_transform = frame_transform or FrameTransform()
        self.adaptive_gripper = adaptive_gripper or AdaptiveGripper(interface, self.calibration)
        self.verbose = verbose

    def _log(self, msg: str):
        if self.verbose:
            print(msg)

    def run(self, waypoints: List[Waypoint]):
        # 1. Read initial joint state and map back to internal angle convention
        raw_current = self.interface.get_current_joint_angles()
        current_angles = self.calibration.commands_to_angles(raw_current)

        for wp in waypoints:
            # 2. Get target coordinate in the local arm frame
            local_x, local_y, local_z = self.frame_transform.world_to_local(wp.x, wp.y, wp.z)
            goal_xyz = (local_x, local_y, local_z)

            # 3. Calculate current Cartesian position using Forward Kinematics
            current_joints = JointAngles(**current_angles)
            start_xyz = self.ik.forward(current_joints)

            # Determine target gripper position for interpolation
            target_gripper = config.GRIPPER_OPEN if wp.gripper_open else current_angles["gripper"]

            # 4. Estimate duration using Cartesian distances
            est_duration = self.planner.estimate_duration(start_xyz, goal_xyz)
            self._log(f"-> {wp.label or '(unlabeled)'}: "
                      f"({wp.x:.3f}, {wp.y:.3f}, {wp.z:.3f}) "
                      f"gripper={'open' if wp.gripper_open else 'closed'} "
                      f"{'[adaptive grasp]' if wp.adaptive_grasp else ''} "
                      f"[~{est_duration:.2f}s]")

            # 5. Plan straight-line trajectory using corrected signatures
            samples = self.planner.plan(
                start_xyz=start_xyz,
                goal_xyz=goal_xyz,
                start_gripper=current_angles["gripper"],
                goal_gripper=target_gripper
            )
            
            # Stream the generated path to CoppeliaSim
            self.interface.stream_trajectory(samples, self.calibration)

            # 6. Update internal tracking angles to match where the trajectory ended
            target_joint_angles = self.ik.solve(local_x, local_y, local_z, wp.gripper_open)
            current_angles = dict(zip(config.JOINT_NAMES, target_joint_angles.as_tuple()))
            current_angles["gripper"] = target_gripper

            # Handle force-feedback closure if requested
            if wp.adaptive_grasp:
                final_gripper_angle = self.adaptive_gripper.close_until_contact(
                    current_angles["gripper"]
                )
                current_angles["gripper"] = final_gripper_angle

            if wp.wait_after_s > 0:
                time.sleep(wp.wait_after_s)

        self._log("Routine complete.")