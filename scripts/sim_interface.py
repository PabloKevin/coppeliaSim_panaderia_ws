"""
sim_interface.py
=================
Thin wrapper around CoppeliaSim's ZMQ Remote API. This is the ONLY module
that talks to the simulator. When you move to the real robot, you write a
sibling module (e.g. serial_interface.py) exposing the same two methods
(connect / stream_trajectory) so routine.py and everything above it does
not need to change at all -- that's the whole point of keeping this
boundary clean, per your "digital twin" goal.

Requires: pip install coppeliasim-zmqremoteapi-client
"""

import time

import config as config


class CoppeliaSimController:
    def __init__(self, joint_paths=config.SIM_JOINT_PATHS):
        self.joint_paths = joint_paths
        self.sim = None
        self.client = None
        self.joint_handles = {}

    def connect(self):
        from coppeliasim_zmqremoteapi_client import RemoteAPIClient

        self.client = RemoteAPIClient()
        self.sim = self.client.require("sim")

        self.joint_handles = {
            name: self.sim.getObject(path)
            for name, path in self.joint_paths.items()
        }
        return self

    def get_current_joint_angles(self) -> dict:
        """Reads the arm's current pose directly from the sim -- used as the
        trajectory start point so moves are always continuous from wherever
        the arm actually is (not assumed)."""
        return {
            name: self.sim.getJointPosition(handle)
            for name, handle in self.joint_handles.items()
        }

    def set_joint_targets(self, joint_commands: dict):
        """Sends one instantaneous set of joint targets (radians, already
        calibrated -- see kinematics.JointCalibration)."""
        for name, value in joint_commands.items():
            self.sim.setJointTargetPosition(self.joint_handles[name], value)

    def stream_trajectory(self, samples, joint_calibration):
        """
        Sends a full pre-planned trajectory (list of trajectory.TrajectorySample)
        to the simulator in real time, pacing each send to match the sample's
        intended timing. joint_calibration is a kinematics.JointCalibration
        instance, applied to every sample before it is sent.
        """
        if not samples:
            return

        start_wall_time = time.perf_counter()
        for sample in samples:
            commands = {
                name: joint_calibration.to_command(name, angle)
                for name, angle in sample.joint_angles.items()
            }
            self.set_joint_targets(commands)

            target_wall_time = start_wall_time + sample.t
            sleep_for = target_wall_time - time.perf_counter()
            if sleep_for > 0:
                time.sleep(sleep_for)

    def step_simulation_if_stepping(self):
        """Optional: call this in the loop instead of time.sleep() if you run
        CoppeliaSim in stepping mode rather than real-time mode. Not used by
        default -- real-time mode with wall-clock pacing (above) is simpler
        and matches how the real serial link will behave."""
        self.client.step()
