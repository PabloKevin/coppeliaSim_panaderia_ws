"""
calibrate_frame.py
===================
Moves the arm through a spread of known cartesian targets and, after each
move, reads the ACTUAL world position of the effector dummy in CoppeliaSim.
Comparing "where we told it to go" vs "where it actually ended up" lets us
fit the arm's true base pose (position + yaw) in the world frame, instead
of guessing at it.

HOW THE FIT WORKS
------------------
For each test point, we already convert the world-frame target into the
robot's LOCAL frame (via the current, possibly-wrong, FrameTransform) before
solving IK. If IK/link-length math is correct, the effector reaches exactly
that LOCAL position -- but the ACTUAL WORLD position it ends up at depends
on the TRUE base pose, not the one currently in config.py. So:

    measured_world = R(true_yaw) @ local_target + true_base_position

Given several (local_target, measured_world) pairs, this is a standard rigid
(rotation + translation) point-set alignment problem, solved here with SVD
(Kabsch algorithm) for the x/y rotation+translation, and a simple mean
offset for z (assuming the base sits flat, i.e. no roll/pitch).

The result REPLACES config.WORLD_BASE_POSITION / WORLD_BASE_YAW directly
(it is not additive with whatever is already there).

Usage:
    1. Add a Dummy object at the gripper center in CoppeliaSim, parented to
       the end effector. Set config.EFFECTOR_DUMMY_PATH to its scene path.
    2. python calibrate_frame.py
    3. Paste the printed WORLD_BASE_POSITION / WORLD_BASE_YAW into config.py
    4. Re-run to confirm the residual error drops close to 0.
"""

import csv
import math
import time

import numpy as np

import config
from kinematics import InverseKinematics, JointCalibration, FrameTransform
from trajectory import QuinticScurvePlanner
from sim_interface import CoppeliaSimController

# Spread of test points across the workspace, in COPPELIASIM WORLD coordinates.
# Edit these to match points you know are safely reachable and visible in
# your scene. More points, more spread out (especially in x/y and in
# different quadrants), gives a better-conditioned fit.
TEST_POINTS = [
    (0.25, 0.00, 0.10),
    (0.30, 0.15, 0.25),
    (0.30, -0.25, 0.25),
    (0.10, 0.15, 0.38),
    (0.20, -0.15, 0.38),
    (0.25, 0.00, 0.35),
    (0.25, -0.10, 0.15),
    (0.15, -0.10, 0.15),
]

SETTLE_TIME_S = 0.5  # pause after each move before reading the dummy position
CSV_PATH = "calibration_data.csv"


def measure_effector(sim_ctrl: CoppeliaSimController):
    handle = sim_ctrl.sim.getObject(config.EFFECTOR_DUMMY_PATH)
    pos = sim_ctrl.sim.getObjectPosition(handle, sim_ctrl.sim.handle_world)
    return tuple(pos)


def fit_rigid_transform_2d(source_xy, target_xy):
    """
    Least-squares rigid (rotation + translation) fit mapping 2D points
    source_xy -> target_xy, via SVD (Kabsch algorithm).
    Returns (yaw_radians, translation_xy).
    """
    src = np.array(source_xy)
    tgt = np.array(target_xy)

    src_mean = src.mean(axis=0)
    tgt_mean = tgt.mean(axis=0)
    src_c = src - src_mean
    tgt_c = tgt - tgt_mean

    H = src_c.T @ tgt_c
    U, _, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:  # reflection guard
        Vt[-1, :] *= -1
        R = Vt.T @ U.T

    t = tgt_mean - R @ src_mean
    yaw = math.atan2(R[1, 0], R[0, 0])
    return yaw, t


def main():
    print("Connecting to CoppeliaSim...")
    sim_ctrl = CoppeliaSimController().connect()
    print("Connected.")

    ik = InverseKinematics()
    planner = QuinticScurvePlanner()
    joint_calibration = JointCalibration()
    frame = FrameTransform()  # uses whatever is CURRENTLY in config.py

    current_angles = sim_ctrl.get_current_joint_angles()

    records = []  # each: dict(world_target, local_target, measured)
    for (wx, wy, wz) in TEST_POINTS:
        lx, ly, lz = frame.world_to_local(wx, wy, wz)
        try:
            target_angles = ik.solve(lx, ly, lz, gripper_open=True)
        except Exception as exc:
            print(f"SKIP world=({wx:.3f},{wy:.3f},{wz:.3f}) local=({lx:.3f},{ly:.3f},{lz:.3f}): {exc}")
            continue

        target_dict = dict(zip(config.JOINT_NAMES, target_angles.as_tuple()))
        samples = planner.plan(current_angles, target_dict)
        sim_ctrl.stream_trajectory(samples, joint_calibration)
        current_angles = target_dict

        time.sleep(SETTLE_TIME_S)
        mx, my, mz = measure_effector(sim_ctrl)

        err = math.dist((wx, wy, wz), (mx, my, mz))
        print(f"world_target=({wx:.3f},{wy:.3f},{wz:.3f})  "
              f"measured=({mx:.3f},{my:.3f},{mz:.3f})  err={err * 1000:.1f} mm")

        records.append({
            "world_target": (wx, wy, wz),
            "local_target": (lx, ly, lz),
            "measured": (mx, my, mz),
            "error_m": err,
        })

    if not records:
        print("No points were reachable -- nothing to fit. Check TEST_POINTS.")
        return

    with open(CSV_PATH, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["world_target_x", "world_target_y", "world_target_z",
                          "local_target_x", "local_target_y", "local_target_z",
                          "measured_x", "measured_y", "measured_z", "error_m"])
        for r in records:
            writer.writerow([*r["world_target"], *r["local_target"], *r["measured"], r["error_m"]])
    print(f"\nSaved raw data to {CSV_PATH}")

    if len(records) < 2:
        print("Need at least 2 reachable points to fit a transform. Add more TEST_POINTS.")
        return

    local_xy = [r["local_target"][:2] for r in records]
    measured_xy = [r["measured"][:2] for r in records]
    yaw, translation_xy = fit_rigid_transform_2d(local_xy, measured_xy)

    z_offset = sum(r["measured"][2] - r["local_target"][2] for r in records) / len(records)

    print("\n=== Fitted base transform (paste into config.py) ===")
    print(f"WORLD_BASE_POSITION = ({translation_xy[0]:.4f}, {translation_xy[1]:.4f}, {z_offset:.4f})")
    print(f"WORLD_BASE_YAW = {yaw:.4f}   # {math.degrees(yaw):.2f} degrees")

    cos_a, sin_a = math.cos(yaw), math.sin(yaw)
    residuals = []
    for r in records:
        lx, ly, lz = r["local_target"]
        pred_x = cos_a * lx - sin_a * ly + translation_xy[0]
        pred_y = sin_a * lx + cos_a * ly + translation_xy[1]
        pred_z = lz + z_offset
        residuals.append(math.dist((pred_x, pred_y, pred_z), r["measured"]))

    rms_mm = 1000 * math.sqrt(sum(e ** 2 for e in residuals) / len(residuals))
    max_mm = 1000 * max(residuals)
    print(f"\nResidual after fit: RMS = {rms_mm:.2f} mm, max = {max_mm:.2f} mm")
    print("(This is the error still left after correcting for base position/yaw --")
    print(" if it's still large, the mismatch may be in link lengths L1/L2/L3/H0/D0")
    print(" rather than (or in addition to) the base pose.)")


if __name__ == "__main__":
    main()
