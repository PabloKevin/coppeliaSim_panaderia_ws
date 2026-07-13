"""
Bakery robot cell controller skeleton (CoppeliaSim ZMQ Remote API)
-------------------------------------------------------------------
Pipeline:
  1. Read the vision sensor image, segment cookies, classify shape.
  2. Only trigger on objects entering a narrow "entry band" (new object).
  3. Convert pixel centroid -> world (x, y) via a pre-computed homography.
  4. Compute angular position theta_obj and radius r_obj relative to the
     conveyor's rotation center.
  5. Using the belt's angular velocity omega, compute the arrival time at
     the fixed strike angle theta_eff.
  6. Schedule the arm to start its push trajectory (t_arrival - travel_time).

Tune every threshold marked TUNE before running on your scene.
"""

import time
import math
import numpy as np
import cv2
import sys
from sim_interface import CoppeliaSimController
from routine import RoutineLoader, RoutineExecutor
import threading

# ---------------------------------------------------------------------------
# Connection & handles
# ---------------------------------------------------------------------------
#client = RemoteAPIClient()
#sim = client.require('sim')
print(f"Connecting to CoppeliaSim...")
sim = CoppeliaSimController().connect()
print("Connected.")



# Measured once geometrically (see calibration notes at bottom of file).
R_PATH = 0.500          # meters, radius of the belt's centerline
LOOP_LENGTH = 2 * math.pi * R_PATH

BRIDGE_TARGETS = {
    'cylinder': '/LeftBridge',
    'cuboid':   '/MiddleBridge',
    'star':     '/RightBridge',
}

SHAPE_IDX = {
    'cylinder': 0,
    'cuboid':   1,
    'star':     2,
}

routine_path = {
    'cuboid': "routines/push_middleBridge.json"
}


# ---------------------------------------------------------------------------
# One-time calibration: pixel -> world homography
# ---------------------------------------------------------------------------
# img_pts: pixel (u, v) of each calibration marker as seen in the vision
# sensor image -- measure these once (e.g. click on a saved frame, or run a
# color-blob detector on the markers themselves) and hard-code them here.
IMG_PTS = np.array([
    [83, 11],      # TUNE: pixel coords of CalibMarker0
    [125, 34],      # TUNE: CalibMarker1
    [8, 118],      # TUNE: CalibMarker2
    [27,124],      # TUNE: CalibMarker3
], dtype=np.float32)

world_pts = np.array(
    [sim.sim.getObjectPosition(m, -1)[:2] for m in sim.calib_markers],
    dtype=np.float32
)
H, _ = cv2.findHomography(IMG_PTS, world_pts)


def pixel_to_world(u, v):
    p = H @ np.array([u, v, 1.0])
    p /= p[2]
    return float(p[0]), float(p[1])


# ---------------------------------------------------------------------------
# Detection & tracking
# ---------------------------------------------------------------------------
ENTRY_BAND_FRAC = 0.2       # TUNE: fraction of image width counted as "entry"
CLASSIFY_DELAY = 0.4        # TUNE (seconds, sim time): wait this long after track starts before classifying
SHOW_DEBUG_WINDOWS = False   # set False once tuned, to remove imshow overhead
 
# TUNE: paste these in from hsv_tune_helper.py
BROWN_LOW  = np.array([3, 43, 105])
BROWN_HIGH = np.array([54, 131, 255])

 
LK_PARAMS = dict(winSize=(21, 21), maxLevel=3,
                  criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01))
 
next_id = 0
scheduled = []       # list of dicts: {id, shape, r, t}
 
prev_mask = None
track_pt = None       # np.array([[u, v]]) or None
track_id = None
track_start_time = None
track_classified = False
 
 
def get_cookie_mask(img_bgr):
    """Binary mask (255 = cookie-colored pixel) via HSV color range."""
    hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, BROWN_LOW, BROWN_HIGH)
    return mask
 
 
def classify_shape(cnt):
    area = cv2.contourArea(cnt)
    if area < 5:
        return None
    peri = cv2.arcLength(cnt, True)
    circularity = 4 * math.pi * area / (peri ** 2 + 1e-6)
    approx = cv2.approxPolyDP(cnt, 0.02 * peri, True)
    hull = cv2.convexHull(cnt)
    hull_area = cv2.contourArea(hull)
    solidity = area / hull_area if hull_area > 0 else 1.0
    #print(f"[classify] area={area:.1f} peri={peri:.1f} circ={circularity:.3f} approx={len(approx)} sol={solidity:.3f}")
 
    if circularity > 0.65:                       # TUNE
        return 'cylinder'
    if len(approx) == 4 and solidity > 0.9:      # TUNE
        return 'cuboid'
    if solidity < 0.8:                          # TUNE
        return 'star'
    return None
 
 
def find_entry_candidate(mask, band_px):
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for cnt in contours:
        M = cv2.moments(cnt)
        if M['m00'] == 0:
            continue
        u, v = M['m10'] / M['m00'], M['m01'] / M['m00']
        if u >= band_px:
            continue
        if cv2.contourArea(cnt) < 5:
            continue
        return u, v
    return None
 
 
def find_contour_near(mask, u, v, max_dist):
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    best, best_d = None, max_dist
    for cnt in contours:
        M = cv2.moments(cnt)
        if M['m00'] == 0:
            continue
        cu, cv_ = M['m10'] / M['m00'], M['m01'] / M['m00']
        d = math.hypot(cu - u, cv_ - v)
        if d < best_d:
            best, best_d = cnt, d
    return best
 
 
def detect_new_objects():
    global next_id, prev_mask, track_pt, track_id, track_start_time, track_classified

    img, res = sim.sim.getVisionSensorImg(sim.vision_sensor)   # NOTE: (image, resolution) order
    img = np.frombuffer(img, dtype=np.uint8).reshape(res[1], res[0], 3)
    img = cv2.flip(img, 0)
    img_bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    mask = get_cookie_mask(img_bgr)
    band_px = res[0] * ENTRY_BAND_FRAC

    if SHOW_DEBUG_WINDOWS:
        disp = cv2.resize(img_bgr, (384, 384), interpolation=cv2.INTER_NEAREST)
        disp_mask = cv2.resize(mask, (384, 384), interpolation=cv2.INTER_NEAREST)
        cv2.imshow('VisionSensor', disp)
        cv2.imshow('CookieMask', disp_mask)
        cv2.waitKey(1)   # REQUIRED for windows to actually refresh on Linux

    if track_pt is None:
        candidate = find_entry_candidate(mask, band_px)
        if candidate is not None:
            u, v = candidate
            track_pt = np.array([[u, v]], dtype=np.float32).reshape(-1, 1, 2)
            track_id = next_id
            next_id += 1
            track_start_time = sim.time()
            track_classified = False
        prev_mask = mask
        return

    # Track with optical flow on the grayscale mask itself (high-contrast,
    # good features for LK since it's pure black/white).
    new_pts, status, _ = cv2.calcOpticalFlowPyrLK(prev_mask, mask, track_pt, None, **LK_PARAMS)
    prev_mask = mask

    if new_pts is None or status is None or status[0][0] == 0:
        track_pt = None
        return

    u, v = new_pts[0, 0]
    if u < 0 or u >= res[0] or v < 0 or v >= res[1]:
        track_pt = None
        return
    track_pt = new_pts

    if not track_classified and sim.time() - track_start_time >= CLASSIFY_DELAY:
        cnt = find_contour_near(mask, u, v, 25)
        if cnt is not None:
            shape = classify_shape(cnt)
            if shape is not None:
                track_classified = True
                x, y = pixel_to_world(u, v)
                r = math.hypot(x - sim.center_pos[0], y - sim.center_pos[1])
                theta_obj = math.atan2(y - sim.center_pos[1], x - sim.center_pos[0])
                register_new_object(track_id, shape, r, theta_obj)
 
 
def register_new_object(oid, shape, r, theta_obj):
    v = sim.get_conveyor_vel()
    if abs(v) < 1e-6:
        return
    dtheta = (sim.theta_eff[SHAPE_IDX[shape]] - theta_obj) % (2 * math.pi)
    ds = dtheta * R_PATH
    t_arrival = sim.time() + ds / abs(v)
    scheduled.append({'id': oid, 'shape': shape, 'r': r, 't': t_arrival})
    print(f"[detect] id={oid} shape={shape} r={r:.3f}m -> strike in {ds/abs(v):.2f}s")

# ---------------------------------------------------------------------------
# Arm execution
# ---------------------------------------------------------------------------
# Measure these empirically: time for the push trajectory from "ready" pose
# to the strike point and back. Can also depend on r (longer reach = longer time).
ARM_TRAVEL_TIME = {'cylinder': 0.6, 'cuboid': 0.6, 'star': 0.6}  # TUNE

executor = RoutineExecutor(interface=sim)

def arm_control_step():
    now = sim.time()
    for job in list(scheduled):
        ready_pose(job['shape'])
        lead = ARM_TRAVEL_TIME.get(job['shape'], 0.6)
        while now < job['t'] - lead:
            time.sleep(0.005)
            now = sim.time()
            continue
        push_object(job['shape'])
        scheduled.remove(job)

def ready_pose(shape):
    waypoints = {
            "waypoints": [
                {"label": "ready_pose", "x": sim.origin_pos[SHAPE_IDX[shape]][0], "y": sim.origin_pos[SHAPE_IDX[shape]][1], "z": sim.origin_pos[SHAPE_IDX[shape]][2], "gripper_open": True, "wait_after_s": 0.0}
            ]
        }
    waypoints = RoutineLoader.from_dict(waypoints)
    executor.run(waypoints)

def push_object(shape):
    #waypoints = RoutineLoader.load(routine_path[shape])
    waypoints = {
            "waypoints": [
                {"label": "push_object", "x": sim.target_pos[SHAPE_IDX[shape]][0], "y": sim.target_pos[SHAPE_IDX[shape]][1], "z": sim.target_pos[SHAPE_IDX[shape]][2], "gripper_open": True, "wait_after_s": 0.0},
                {"label": "home", "x": sim.origin_pos[SHAPE_IDX[shape]][0], "y": sim.origin_pos[SHAPE_IDX[shape]][1], "z": sim.origin_pos[SHAPE_IDX[shape]][2], "gripper_open": True, "wait_after_s": 0.0}
            ]
        }
    waypoints = RoutineLoader.from_dict(waypoints)
    executor.run(waypoints)



# ---------------------------------------------------------------------------
# Throughput optimization (optional)
# ---------------------------------------------------------------------------
# Ceiling: two cookies closer together (in arc length) than the arm's full
# cycle time will cause a miss. If you know/measure the minimum spacing
# s_min between consecutive cookies on the belt, cap speed accordingly:
#
#   T_CYCLE = 1.2          # TUNE: measured full pick+push+retract time (s)
#   S_MIN   = 0.15         # TUNE: closest expected spacing between cookies (m)
#   v_max = S_MIN / T_CYCLE
#   set_conveyor_vel(min(v_max * 0.9, YOUR_DESIRED_SPEED))  # 10% safety margin

# ---------------------------------------------------------------------------
# Main loop
# ---------------------------------------------------------------------------
sim.sim.startSimulation()
time.sleep(0.5)

try:
    while True:
        detect_new_objects()
        arm_control_step()
        time.sleep(0.01)
except KeyboardInterrupt:
    sim.stopSimulation()