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
import random

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


class ObjectGenerator():
    def __init__(self):
        self.sim = None
        self.n_objects = 0

    def connect(self):
        from coppeliasim_zmqremoteapi_client import RemoteAPIClient

        self.client = RemoteAPIClient()
        self.sim = self.client.require("sim")

        #galletas
        self.galletas_spawn = self.sim.getObject('/galletas_spawn')
        self.galletas_spawn_pos = self.sim.getObjectPosition(self.galletas_spawn, -1)


    def get_objects_by_prefix(self, prefix):
        """
        Return a list of (handle, alias, position) for every object in the
        scene whose name/alias starts with `prefix`.
        """
        all_handles = self.sim.getObjectsInTree(self.sim.handle_scene, self.sim.handle_all, 0)
    
        matches = []
        for h in all_handles:
            alias = self.sim.getObjectAlias(h)
            if alias.startswith(prefix):
                pos = self.sim.getObjectPosition(h, -1)  # world frame
                matches.append((h, alias, pos))
        self.n_objects = len(matches)
    
        return matches
 

    def is_position_free(self, candidate_pos, prefix="Galleta", min_distance=0.1):
        """
        Check that no existing object whose name starts with `prefix` is
        within `min_distance` meters of candidate_pos (x, y, z).
        Returns True if the spot is free, False if a "Galleta" is too close.
        """
        existing = self.get_objects_by_prefix(prefix)
    
        for handle, alias, pos in existing:
            dx = pos[0] - candidate_pos[0]
            dy = pos[1] - candidate_pos[1]
            dz = pos[2] - candidate_pos[2]
            dist = (dx ** 2 + dy ** 2 + dz ** 2) ** 0.5
    
            if dist < min_distance:
                print(f"  -> Blocked: {alias} is {dist:.3f} m away "
                    f"(min allowed {min_distance} m)")
                return False
    
        return True
    
    
    def get_template_handles(self):
        """Look up the handles of the three template objects by name."""
        handles = []
        for name in config.GALLETAS_NAMES:
            h = self.sim.getObject(name)
            handles.append(h)
        return handles
    
    
    def spawn_object(self, template_handle, position, index):
        """
        Duplicate a template object (mesh, mass, dynamics, everything)
        and place the copy at the given position.
        """
        # copyPasteObjects duplicates the object (and any children/scripts)
        # exactly, including physical properties. Returns a list of new
        # handles (one per input handle, plus any children).
        new_handles = self.sim.copyPasteObjects([template_handle], 0)
        new_handle = new_handles[0]
    
        # Give the copy a readable, unique name
        orig_alias = self.sim.getObjectAlias(template_handle)
        self.sim.setObjectAlias(new_handle, f"{orig_alias}_copy{index}")
    
        # Place it in the world frame (-1 = absolute/world)
        self.sim.setObjectPosition(new_handle, -1, position)
    
        # If the copy is dynamic, reset its velocity/forces so it doesn't
        # inherit any leftover motion from the template
        try:
            self.sim.resetDynamicObject(new_handle)
        except Exception:
            pass  # not a dynamic object, safe to ignore
    
        print(f"[{time.strftime('%H:%M:%S')}] Spawned {orig_alias}_copy{index} "
            f"at {position}")
    
        return new_handle

    def tarea_spawn_galletas(self):
        self.connect()

        template_handles = self.get_template_handles()
        print(f"Found {len(template_handles)} template objects.")
    
        count = 0
        try:
            while True:
                if count < config.MAX_OBJECTS:
                    template = random.choice(template_handles)
        
                    if self.is_position_free(self.galletas_spawn_pos, prefix="Galleta",
                                        min_distance=config.MIN_SPAWN_DISTANCE):
                        self.spawn_object(template, self.galletas_spawn_pos, count)
                        count += 1
                    else:
                        print("  -> Skipping spawn, position occupied.")
    
                time.sleep(random.uniform(1.0,5.0))
    
        except KeyboardInterrupt:
            print("\nStopped by user.")
    