"""
measure_geometry.py
====================
Reads joint/dummy positions directly from CoppeliaSim and computes the link
geometry constants (L1, L2, L3, H0, D0) that kinematics.py assumes, instead
of relying on manual tape-measure readings. This removes one whole source
of error before we even get to base-pose calibration.

WHY WORLD-FRAME DISTANCE IS SAFE FOR L1/L2/L3
------------------------------------------------
Euclidean distance between two points is invariant to rotation and
translation of the whole system. So even if the arm is currently posed at
some odd angle, or the base is mounted at a diagonal, dist(pointA, pointB)
in WORLD coordinates still gives the true rigid-body link length. No need
to project onto any particular axis for these.

WHY D0/H0 NEED A DIFFERENT APPROACH
--------------------------------------
D0 (radial offset) and H0 (vertical offset) are NOT rotation-invariant --
they're defined relative to the base joint's OWN local frame (D0 = offset
in the direction the arm points at theta_base=0, H0 = straight up from
there). We get this "for free" and robustly by asking CoppeliaSim for the
shoulder's position "relative to" the base joint object -- this automatically
un-rotates by whatever the base joint's current angle is, so it works no
matter what pose the arm happens to be in when you run this script.

Usage:
    python measure_geometry.py
"""

import math

import config


def main():
    from coppeliasim_zmqremoteapi_client import RemoteAPIClient

    client = RemoteAPIClient()
    sim = client.require("sim")

    base_h = sim.getObject(config.SIM_JOINT_PATHS["base"])
    shoulder_h = sim.getObject("/EZZYbotARM_Mk2/Joint2_Dummy")
    aux1_h = sim.getObject("/EZZYbotARM_Mk2/Joint3_Dummy")
    aux2_h = sim.getObject("/EZZYbotARM_Mk2/Joint4_Dummy")
    end_effector_h = sim.getObject("/EZZYbotARM_Mk2/EndEffector_Dummy")

    def world_pos(handle):
        return sim.getObjectPosition(handle, sim.handle_world)

    def dist(h_a, h_b):
        pa = world_pos(h_a)
        pb = world_pos(h_b)
        return math.dist(pa, pb)

    # --- Link lengths: plain world-frame distance, rotation-invariant ---
    L1 = dist(shoulder_h, aux1_h)
    L2 = dist(aux1_h, aux2_h)
    L3 = dist(aux2_h, end_effector_h)

    # --- Base offset: shoulder position relative to the BASE's own frame,
    #     which cancels out the base joint's current rotation automatically.
    shoulder_rel_to_base = sim.getObjectPosition(shoulder_h, base_h)
    d0 = math.hypot(shoulder_rel_to_base[0], shoulder_rel_to_base[1])
    h0 = shoulder_rel_to_base[2]

    print("=== Raw world positions (for sanity-checking / debugging) ===")
    print(f"  base (motor1):        {world_pos(base_h)}")
    print(f"  shoulder (motor2):    {world_pos(shoulder_h)}")
    print(f"  auxMotor1:            {world_pos(aux1_h)}")
    print(f"  auxMotor2:            {world_pos(aux2_h)}")
    print(f"  EndEffector_Dummy:    {world_pos(end_effector_h)}")

    print("\n=== Shoulder position relative to base's own frame ===")
    print(f"  {shoulder_rel_to_base}  (x,y should mostly be one axis; "
          f"z is the vertical component)")

    print("\n=== Measured geometry (paste into config.py) ===")
    print(f"L1 = {L1:.5f}   # shoulder -> elbow hinge (was {config.L1:.5f})")
    print(f"L2 = {L2:.5f}   # elbow hinge -> gripper mount aux joint (was {config.L2:.5f})")
    print(f"L3 = {L3:.5f}   # gripper mount aux joint -> gripper center (was {config.L3:.5f})")
    print(f"H0 = {h0:.5f}   # base -> shoulder, vertical (was {config.H0:.5f})")
    print(f"D0 = {d0:.5f}   # base -> shoulder, radial (was {config.D0:.5f})")

    print("\nNOTE: EndEffector_Dummy already exists in your scene at "
          "'/EZZYbotARM_Mk2/EndEffector_Dummy'. Update "
          "config.EFFECTOR_DUMMY_PATH to that path (instead of creating a "
          "new dummy) before running calibrate_frame.py.")


if __name__ == "__main__":
    main()
