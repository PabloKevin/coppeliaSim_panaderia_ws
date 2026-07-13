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

import math
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
        self.vision_sensor    = self.sim.getObject('/visionSensor')
        self.conveyor_handle  = self.sim.getObject('/conveyorSystem')     # TUNE: exact name/path of your conveyor model
        self.conveyor_center  = self.sim.getObject('/conveyorSystem/ConveyorCenter')     # dummy at the ring's rotation axis
        self.origin_dummies     = [self.sim.getObject('/LeftOrigin'), self.sim.getObject('/MiddleOrigin') , self.sim.getObject('/RightOrigin')]     # dummy at the fixed angle the arm pushes from
        self.strike_dummies     = [self.sim.getObject('/LeftStrike'), self.sim.getObject('/MiddleStrike'), self.sim.getObject('/RightStrike')]        # dummy at the strike point on the belt's centerline
        self.target_dummies     = [self.sim.getObject('/LeftTarget'), self.sim.getObject('/MiddleTarget'), self.sim.getObject('/RightTarget')]        # IK target dummy for the arm tip
        self.calib_markers    = [self.sim.getObject('/CalibMarker1'), self.sim.getObject('/CalibMarker2'), self.sim.getObject('/CalibMarker3'), self.sim.getObject('/CalibMarker4')]  # TUNE: order of markers in IMG_PTS
        #geometry
        self.center_pos = self.sim.getObjectPosition(self.conveyor_center, -1)
        self.origin_pos = [self.sim.getObjectPosition(self.origin_dummies[0], -1), self.sim.getObjectPosition(self.origin_dummies[1], -1), self.sim.getObjectPosition(self.origin_dummies[2], -1)]
        self.strike_pos = [self.sim.getObjectPosition(self.strike_dummies[0], -1), self.sim.getObjectPosition(self.strike_dummies[1], -1), self.sim.getObjectPosition(self.strike_dummies[2], -1)]
        self.target_pos = [self.sim.getObjectPosition(self.target_dummies[0], -1), self.sim.getObjectPosition(self.target_dummies[1], -1), self.sim.getObjectPosition(self.target_dummies[2], -1)]
        self.theta_eff = [math.atan2(self.strike_pos[i][1] - self.center_pos[1], self.strike_pos[i][0] - self.center_pos[0]) for i in range(3)]
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

    def get_conveyor_vel(self):
        """Current linear belt speed (m/s), read live from the conveyor model's state."""
        state = self.sim.readCustomTableData(self.conveyor_handle, '__state__')
        return state['vel']


    def set_conveyor_vel(self, v):
        """Command the belt's linear speed (m/s) via velocity control."""
        self.sim.setBufferProperty(self.conveyor_handle, 'customData.__ctrl__', self.sim.packTable({'vel': v}))

    def time(self):
        """Current sim time (seconds) -- useful for scheduling future events."""
        return self.sim.getSimulationTime()
