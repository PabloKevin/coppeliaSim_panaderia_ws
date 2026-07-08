"""
grasp.py
========
Closed-loop grasp control using a CoppeliaSim force sensor placed between
the gripper's driven joint and the finger tip.

WHY THIS IS NEEDED (not just "nice to have")
---------------------------------------------
The gripper joint is KINEMATIC: it's driven directly to a target position
every step, ignoring any contact reaction force. A normal dynamically
simulated joint would stall against an obstacle once its motor torque limit
is reached; a kinematic joint has no such limit -- it will happily keep
commanding the finger geometry straight through a solid object. That
mismatch is exactly what caused the "object launched like a bullet"
behavior: the solver has to resolve a growing interpenetration between the
finger and the cylinder in one step, and injects a large corrective impulse
into the (dynamically simulated) cylinder to shove it back out.

The fix: close the gripper in small steps, and after each step read a force
sensor placed between the driven joint and the fingertip. Stop the instant
the measured contact force crosses a threshold. This is also a reasonable
model of what you'd eventually do on the real robot with current-sensing
servos (current spike = contact), so it's not just a simulation workaround.

SCENE SETUP REQUIRED
---------------------
Insert a Force Sensor object (Add -> Force sensor in CoppeliaSim) in the
kinematic chain between the driven gripper joint and the actual finger-tip
geometry that touches the object, then set config.FORCE_SENSOR_PATH to its
scene path. This works even though the joint itself is kinematic -- the
force sensor still reports the true contact reaction force, which is a
separate physics computation from the joint's own driven motion.
"""

import math
import time

import config


class AdaptiveGripper:
    """
    Closes the gripper in small steps, checking a force sensor after each
    one, and stops as soon as contact is detected (or a safety limit/timeout
    is hit). Used by RoutineExecutor whenever a waypoint sets
    adaptive_grasp=True.
    """

    def __init__(self, interface, calibration, force_sensor_path=config.GRIPPER_FORCE_SENSOR_PATH):
        self.interface = interface
        self.calibration = calibration
        self.force_sensor_path = force_sensor_path
        self._sensor_handle = None

    def _get_sensor_handle(self):
        if self._sensor_handle is None:
            self._sensor_handle = self.interface.sim.getObject(self.force_sensor_path)
        return self._sensor_handle

    def _read_force_magnitude(self) -> float:
        handle = self._get_sensor_handle()
        state, force_vec, _torque_vec = self.interface.sim.readForceSensor(handle)
        if not state:
            return -1.0
        force = force_vec[1]
        #force = math.sqrt(sum(f ** 2 for f in force_vec))
        print(f"Force magnitude {force:.7f} N")
        return force
    
    def _read_distance_magnitude(self) -> float:
        handle = self._get_sensor_handle()
        state, distance, detectedPoint, detectedObjectHandle, detectedSurfaceNormal = self.interface.sim.readProximitySensor(handle)
        if not state:
            return -1.0
        #print(f"Distance {distance:.3f} m")
        return distance

    def close_until_contact(self, current_gripper_internal_angle: float,
                             force_threshold_n=config.GRIP_FORCE_THRESHOLD_N,
                             distance_threshold_m=config.GRIP_DISTANCE_THRESHOLD_m,
                             fully_closed_internal=None,
                             step_rad=config.GRIP_STEP_RAD,
                             step_dt=config.GRIP_STEP_DELAY_S,
                             timeout_s=config.GRIPPER_CLOSE_TIMEOUT_S) -> float:
        """
        current_gripper_internal_angle: where the gripper actually is right
        now (internal-angle convention), so this call is safe to make
        regardless of what happened before it.

        Stops as soon as:
          - force_threshold_n is exceeded (object gripped) -- the normal case
          - the gripper reaches config.GRIPPER_CLOSED (nothing there to grip)
          - timeout_s elapses (safety fallback, shouldn't normally trigger)

        Returns the INTERNAL gripper angle actually achieved. Do NOT assume
        it equals config.GRIPPER_CLOSED -- normally it will stop earlier,
        exactly at the object's surface.
        """
        if fully_closed_internal is None:
            fully_closed_internal = config.GRIPPER_CLOSED

        internal_angle = current_gripper_internal_angle
        direction = 1.0 if fully_closed_internal > internal_angle else -1.0
        step = abs(step_rad) * direction

        start_time = time.time()
        contact_detected = False

        while True:
            reached_limit = (
                (direction > 0 and internal_angle >= fully_closed_internal) or
                (direction < 0 and internal_angle <= fully_closed_internal)
            )
            if reached_limit or (time.time() - start_time > timeout_s):
                break

            internal_angle += step
            if direction > 0:
                internal_angle = min(internal_angle, fully_closed_internal)
            else:
                internal_angle = max(internal_angle, fully_closed_internal)

            raw_command = self.calibration.to_command("gripper", internal_angle)
            self.interface.set_joint_targets({"gripper": raw_command})
            time.sleep(step_dt)

            """if self._read_distance_magnitude() <= distance_threshold_m and self._read_distance_magnitude() != -1.0:
                contact_detected = True
                break"""
            if self._read_force_magnitude() >= force_threshold_n and self._read_force_magnitude() != -1.0:
                contact_detected = True
                break

        if not contact_detected:
            print("WARNING: gripper reached its closing limit without detecting "
                  "contact force -- check config.FORCE_SENSOR_PATH, "
                  "GRIP_FORCE_THRESHOLD_N, or whether the object was actually "
                  "within the gripper's jaws.")

        return internal_angle