# CoppeliaSim Panadería Workspace

Cartesian control pipeline for a uArm-style 4-DOF robot arm (EEZYbotARM Mk2),
simulated in CoppeliaSim and designed to transfer to a real arm as a digital
twin. You define pick-and-place routines in cartesian coordinates; the
pipeline converts them to jerk-limited joint trajectories and streams them
to the sim (and, later, to real servos over serial).

## Requirements for CoppeliaSim control with Python

```bash
pip install coppeliasim-zmqremoteapi-client numpy
```

CoppeliaSim 4.x, scene loaded, simulation **running** (Play button) before
running any script below.

## The 4 files you need to get started

- **`config.py`** — every tunable constant: arm geometry, joint limits,
  velocity/acceleration limits, and calibration (world-frame position/yaw,
  per-joint sign+offset, gripper force threshold). **This is the file you
  edit most.**
- **`make_routine.py`** — generates a routine JSON (a sequence of cartesian
  waypoints) from a few named parameters, instead of hand-writing JSON.
  Edit the parameters at the bottom (`__main__` block) and run it.
- **`main.py`** — connects to CoppeliaSim and runs a routine JSON through
  the full pipeline (IK → jerk-limited trajectory → stream to sim).
- **`calibrate_frame.py`** — measures the arm's real base position/yaw in
  the world frame and fits the correct `config.py` values, instead of you
  guessing them.

## Quick start

```bash
# 1. Generate a routine (edit the parameters inside make_routine.py first)
python make_routine.py
# -> writes e.g. pick_and_place_02.json

# 2. Run it in CoppeliaSim (sim must already be playing)
python main.py pick_and_place_02.json
```

`make_routine.py`'s `WaypointGenerator` currently supports:
- `pick_and_place_waypoints(...)` — home → approach → grip (force-based,
  `adaptive_grasp: True`) → lift → place → release → home. Set `loop_reps`
  > 0 to automatically repeat the pick-and-place back and forth that many
  times.
- `push_object_waypoints(...)` — a simpler two-point push motion.
- `unite_waypoints([...])` — concatenate several generated routines into one
  JSON before saving.

## If it's not moving to the right place (new sim, new arm, or after any physical change)

**This is almost always a calibration problem, not a bug.** Every number
that makes cartesian targets land in the right physical spot lives in
`config.py`, and every one of them is specific to one physical arm + one
scene setup. If you swap arms, rebuild the scene, or move the robot's mount,
recalibrate in this order:

1. **Geometry** (`L1, L2, L3, L3_FIXED_ANGLE, H0, D0`) — link lengths and
   offsets. Wrong here means the whole reachable workspace is shifted or
   distorted, no matter how well the frame/joints are calibrated.
2. **Joint calibration** (`CALIBRATION` dict: `sign` + `offset` per joint,
   plus `JOINT_LIMITS`) — maps our internal angle convention to what the
   simulated/real joint actually expects. Wrong sign = joint moves the
   opposite direction; wrong offset = consistent angular error at every pose.
3. **World frame** (`WORLD_BASE_POSITION`, `WORLD_BASE_YAW`) — where the
   arm's base actually sits in the scene/world. Run `calibrate_frame.py`
   last, after 1 and 2 are correct, since it fits this by measuring the
   residual error after everything else — if you run it first, it'll just
   be compensating for the wrong things.
4. **Grip force threshold** (`GRIP_FORCE_THRESHOLD_N` and friends) — if
   grasping is too weak (object slips) or too aggressive (object launches),
   this needs re-tuning per object/gripper material. `grasp.py` prints the
   force reading at every closing step so you can see exactly where contact
   is actually happening and set the threshold just above the noise floor.

**Symptom → likely cause:**
- Arm reaches the wrong spot, but consistently offset in the same direction
  → world frame (`WORLD_BASE_POSITION`/`WORLD_BASE_YAW`) or `D0`/`H0`.
- Arm reaches a spot scaled wrong (overshoots far targets, undershoots near
  ones, or vice versa) → link lengths (`L1`/`L2`/`L3`).
- A joint moves the wrong direction, or the arm folds into itself /
  hits a limit immediately → `CALIBRATION[joint]["sign"]` or `ELBOW_CONFIG`.
- Right position but wrong angle/orientation feel → `CALIBRATION[joint]["offset"]`.
- Grasp crushes or launches the object → `GRIP_FORCE_THRESHOLD_N` too high,
  or `GRIP_STEP_RAD` too coarse (closes too fast to react to contact).

Run `calibrate_frame.py` any time after a recalibration pass — it reports a
residual RMS error in mm. A few mm is normal simulation-and-servo-tolerance
noise; tens of mm or more means one of the steps above still needs work.

## Other modules in the repo (not usually edited directly)

- `kinematics.py` — analytical inverse/forward kinematics and the
  world-frame ↔ local-frame transform.
- `trajectory.py` — the jerk-limited (quintic S-curve) motion planner.
- `sim_interface.py` — CoppeliaSim ZMQ Remote API wrapper.
- `routine.py` — loads a routine JSON and executes it as a waypoint state
  machine, tying IK + trajectory + sim streaming together.
- `grasp.py` — force-sensor-based closed-loop grasping (`adaptive_grasp`
  waypoints).
- `measure_geometry.py`, `characterize_linkage.py` — one-off diagnostic
  scripts that measure `L1/L2/L3/H0/D0` and the parallel-linkage behavior
  directly from the CoppeliaSim scene, instead of a tape measure.