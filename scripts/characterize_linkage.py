"""
characterize_linkage.py
========================
Determines how the passive joint at auxMotor2 (between L2 and L3) actually
behaves, instead of assuming it's a fixed weld (colinear with L2).

METHOD
------
Directly commands a grid of (shoulder, elbow) angle combinations -- bypassing
IK entirely -- holding base and gripper fixed. At each pose, reads the world
position of shoulder/auxMotor1/auxMotor2/EndEffector_Dummy and computes the
ACTUAL absolute angle of each segment (L1, L2, L3) in the arm's vertical
plane (base held at 0, so world x/z serve directly as the plane axes).

We then fit:
    theta3_actual = a * theta1_actual + b * theta2_actual + c

via least squares. The coefficients (a, b, c) tell us exactly what the
passive linkage at auxMotor2 is doing mechanically -- see the printed
interpretation guide at the end.

NOTE: this bypasses the trajectory planner and commands raw joint targets
directly with only a fixed settle time, so the arm may move somewhat
abruptly between grid points. That's fine for this one-off measurement.

Usage:
    python characterize_linkage.py
"""

import math
import time

import numpy as np

import config


def main():
    from coppeliasim_zmqremoteapi_client import RemoteAPIClient

    client = RemoteAPIClient()
    sim = client.require("sim")

    base_j = sim.getObject(config.SIM_JOINT_PATHS["base"])
    shoulder_j = sim.getObject(config.SIM_JOINT_PATHS["shoulder"])
    elbow_j = sim.getObject(config.SIM_JOINT_PATHS["elbow"])
    gripper_j = sim.getObject(config.SIM_JOINT_PATHS["gripper"])
    aux1_h = sim.getObject("/EZZYbotARM_Mk2/auxMotor1")
    aux2_h = sim.getObject("/EZZYbotARM_Mk2/auxMotor2")
    ee_h = sim.getObject("/EZZYbotARM_Mk2/EndEffector_Dummy")

    def world_pos(h):
        return sim.getObjectPosition(h, sim.handle_world)

    sim.setJointTargetPosition(base_j, 0.0)
    sim.setJointTargetPosition(gripper_j, config.GRIPPER_CLOSED)
    time.sleep(0.5)

    # Quick base sign check: command a small positive base angle, see which
    # way the END EFFECTOR (far from the rotation axis -- the shoulder
    # itself has near-zero radial offset D0, so it's a bad choice here)
    # actually swings in world y.
    base_test_angle = 0.3
    sim.setJointTargetPosition(base_j, base_test_angle)
    time.sleep(0.8)
    p_ee_test = world_pos(ee_h)
    sim.setJointTargetPosition(base_j, 0.0)
    time.sleep(0.8)
    p_ee_home = world_pos(ee_h)
    dy = p_ee_test[1] - p_ee_home[1]
    print(f"Base sign check: commanded base={base_test_angle:+.2f} rad -> "
          f"end effector moved dy={dy:+.4f} m in world y "
          f"({'consistent with sign=+1' if dy > 0 else 'suggests sign=-1 needed'})")
    print()

    shoulder_lo, shoulder_hi = config.JOINT_LIMITS["shoulder"]
    elbow_lo, elbow_hi = config.JOINT_LIMITS["elbow"]

    # Stay inside the middle 70% of each range to avoid mechanical limits/binding
    shoulder_vals = np.linspace(shoulder_lo * 0.7, shoulder_hi * 0.7, 4)
    elbow_vals = np.linspace(elbow_lo * 0.7, elbow_hi * 0.7, 4)

    SETTLE_S = 0.6
    rows = []

    for th1_cmd in shoulder_vals:
        for th2_cmd in elbow_vals:
            sim.setJointTargetPosition(shoulder_j, float(th1_cmd))
            sim.setJointTargetPosition(elbow_j, float(th2_cmd))
            time.sleep(SETTLE_S)

            p_shoulder = world_pos(shoulder_j)
            p_aux1 = world_pos(aux1_h)
            p_aux2 = world_pos(aux2_h)
            p_ee = world_pos(ee_h)

            theta1_actual = math.atan2(p_aux1[2] - p_shoulder[2], p_aux1[0] - p_shoulder[0])
            theta2_actual = math.atan2(p_aux2[2] - p_aux1[2], p_aux2[0] - p_aux1[0])
            theta3_actual = math.atan2(p_ee[2] - p_aux2[2], p_ee[0] - p_aux2[0])

            rows.append({
                "shoulder_cmd": th1_cmd, "elbow_cmd": th2_cmd,
                "theta1_actual": theta1_actual, "theta2_actual": theta2_actual,
                "theta3_actual": theta3_actual,
            })
            print(f"cmd(shoulder={th1_cmd:+.3f}, elbow={th2_cmd:+.3f})  ->  "
                  f"theta1={math.degrees(theta1_actual):+7.2f}deg  "
                  f"theta2={math.degrees(theta2_actual):+7.2f}deg  "
                  f"theta3={math.degrees(theta3_actual):+7.2f}deg")

    # Sanity check: does the shoulder servo set L1's ABSOLUTE angle directly
    # (slope should be close to +1 if so -- but don't assume, the elbow
    # turned out to be inverted, so verify this one too).
    shoulder_cmds = np.array([r["shoulder_cmd"] for r in rows])
    theta1_actuals = np.array([r["theta1_actual"] for r in rows])
    shoulder_slope_check = np.polyfit(shoulder_cmds, theta1_actuals, 1)
    print(f"theta1_actual vs shoulder_cmd: slope={shoulder_slope_check[0]:.3f}, "
          f"intercept={math.degrees(shoulder_slope_check[1]):.2f}deg")

    # Sanity check: does the elbow servo really set L2's ABSOLUTE angle
    # directly, as kinematics.py currently assumes? (slope of theta2_actual
    # vs elbow_cmd should be close to 1 if so)
    elbow_cmds = np.array([r["elbow_cmd"] for r in rows])
    theta2_actuals = np.array([r["theta2_actual"] for r in rows])
    slope_check = np.polyfit(elbow_cmds, theta2_actuals, 1)
    print(f"theta2_actual vs elbow_cmd: slope={slope_check[0]:.3f}, "
          f"intercept={math.degrees(slope_check[1]):.2f}deg "
          f"(slope near 1.0 confirms elbow servo sets L2's absolute angle directly)")

    # Main fit: theta3 = a*theta1 + b*theta2 + c
    A = np.array([[r["theta1_actual"], r["theta2_actual"], 1.0] for r in rows])
    y = np.array([r["theta3_actual"] for r in rows])
    coeffs, _, _, _ = np.linalg.lstsq(A, y, rcond=None)
    a, b, c = coeffs
    pred = A @ coeffs
    rms_deg = math.degrees(math.sqrt(np.mean((pred - y) ** 2)))

    print("\n=== Fit: theta3_actual = a * theta1_actual + b * theta2_actual + c ===")
    print(f"a = {a:.4f}")
    print(f"b = {b:.4f}")
    print(f"c = {c:.4f}  ({math.degrees(c):.2f} degrees)")
    print(f"Fit RMS residual: {rms_deg:.3f} degrees  (small = the linear model explains it well)")

    print("\n=== How to read this ===")
    print("a~0, b~0        : L3 has a FIXED world orientation (classic parallelogram,")
    print("                  gripper always points the same way). Use c as that fixed angle.")
    print("a~0, b~-1       : L3's angle exactly cancels the elbow's rotation --")
    print("                  net (L2+L3) direction is fixed relative to the SHOULDER, not ground.")
    print("a~-1, b~0       : L3's angle exactly cancels the shoulder's rotation.")
    print("Something else  : read a, b, c directly -- send them to me and I'll fold the exact")
    print("                  relationship into kinematics.py's forward/inverse equations.")


if __name__ == "__main__":
    main()