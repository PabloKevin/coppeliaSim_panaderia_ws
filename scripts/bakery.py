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
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

# ---------------------------------------------------------------------------
# Connection & handles
# ---------------------------------------------------------------------------
client = RemoteAPIClient()
sim = client.require('sim')

vision_sensor    = sim.getObject('/visionSensor')
conveyor_handle  = sim.getObject('/conveyorSystem')     # TUNE: exact name/path of your conveyor model
conveyor_center  = sim.getObject('/conveyorSystem/ConveyorCenter')     # dummy at the ring's rotation axis
strike_dummy     = sim.getObject('/MiddleOrigin')        # dummy at the fixed angle the arm pushes from
ik_target        = sim.getObject('/MiddleTarget')        # IK target dummy for the arm tip
calib_markers    = [sim.getObject('/CalibMarker1'), sim.getObject('/CalibMarker2'), sim.getObject('/CalibMarker3'), sim.getObject('/CalibMarker4')]  # TUNE: order of markers in IMG_PTS

# Measured once geometrically (see calibration notes at bottom of file).
R_PATH = 0.500          # meters, radius of the belt's centerline
LOOP_LENGTH = 2 * math.pi * R_PATH

BRIDGE_TARGETS = {
    'cylinder': '/Bridge1',
    'cuboid':   '/Bridge2',
    'star':     '/Bridge3',
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
    [sim.getObjectPosition(m, -1)[:2] for m in calib_markers],
    dtype=np.float32
)
H, _ = cv2.findHomography(IMG_PTS, world_pts)


def pixel_to_world(u, v):
    p = H @ np.array([u, v, 1.0])
    p /= p[2]
    return float(p[0]), float(p[1])


# ---------------------------------------------------------------------------
# Geometry
# ---------------------------------------------------------------------------
center_pos = sim.getObjectPosition(conveyor_center, -1)
strike_pos = sim.getObjectPosition(strike_dummy, -1)
theta_eff = math.atan2(strike_pos[1] - center_pos[1], strike_pos[0] - center_pos[0])


def get_conveyor_vel():
    """Current linear belt speed (m/s), read live from the conveyor model's state."""
    state = sim.readCustomTableData(conveyor_handle, '__state__')
    return state['vel']


def set_conveyor_vel(v):
    """Command the belt's linear speed (m/s) via velocity control."""
    sim.setBufferProperty(conveyor_handle, 'customData.__ctrl__', sim.packTable({'vel': v}))


# ---------------------------------------------------------------------------
# Detection & shape classification
# ---------------------------------------------------------------------------
next_id = 0
scheduled = []    # list of dicts: {id, shape, r, t}
 
ENTRY_BAND_FRAC = 0.2     # TUNE: fraction of image width counted as "entry" side
BIN_THRESH = 150          # TUNE: grayscale threshold to separate cookie from belt
 
# Optical-flow track state. Only ONE active track at a time is supported here,
# matching a single-lane entry point. A new detection is only allowed to start
# once the previous track has been lost (point exited frame / flow broke down).
LK_PARAMS = dict(winSize=(21, 21), maxLevel=3,
                  criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01))
 
prev_gray = None
track_pt = None       # np.array([[u, v]], dtype=float32) or None if no active track
track_shape = None
 
 
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
 
    if circularity < 0.5 and len(approx)==4:                       # TUNE
        return 'cylinder'
    if len(approx) == 4 and solidity > 0.9:      # TUNE
        return 'cuboid'
    if solidity < 0.75:                          # TUNE
        return 'star'
    return None
 
 
def find_entry_candidate(gray, band_px):
    """Contour-based search, restricted to the entry band, used only to seed a
    new optical-flow track when there isn't one active yet."""
    _, mask = cv2.threshold(gray, BIN_THRESH, 255, cv2.THRESH_BINARY)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for cnt in contours:
        M = cv2.moments(cnt)
        if M['m00'] == 0:
            continue
        u, v = M['m10'] / M['m00'], M['m01'] / M['m00']
        if u >= band_px:
            continue
        shape = classify_shape(cnt)
        if shape is None:
            continue
        return u, v, shape
    return None
 
 
def detect_new_objects():
    global next_id, prev_gray, track_pt, track_shape
 
    img, res = sim.getVisionSensorImg(vision_sensor)   
    img = np.frombuffer(img, dtype=np.uint8).reshape(res[1], res[0], 3)
    img = cv2.flip(img, 0)  # CoppeliaSim images are bottom-up
    gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
    band_px = res[0] * ENTRY_BAND_FRAC   # TUNE: depends on camera orientation
 
    if track_pt is None:
        # No active track: look for a new object entering the band.
        candidate = find_entry_candidate(gray, band_px)
        if candidate is not None:
            time.sleep(0.2)
            candidate = find_entry_candidate(gray, band_px)
            u, v, shape = candidate
            oid = next_id
            next_id += 1
            x, y = pixel_to_world(u, v)
            r = math.hypot(x - center_pos[0], y - center_pos[1])
            theta_obj = math.atan2(y - center_pos[1], x - center_pos[0])
            register_new_object(oid, shape, r, theta_obj)
 
            # Seed the optical-flow track so we don't re-detect this same
            # object again until it's left the frame.
            track_pt = np.array([[u, v]], dtype=np.float32).reshape(-1, 1, 2)
            track_shape = shape
        prev_gray = gray
        return
 
    # An active track exists: propagate it with optical flow instead of
    # re-running contour detection (robust to segmentation flicker).
    new_pts, status, _ = cv2.calcOpticalFlowPyrLK(prev_gray, gray, track_pt, None, **LK_PARAMS)
    prev_gray = gray
 
    if new_pts is None or status is None or status[0][0] == 0:
        # Flow lost -- assume the object left the frame or tracking broke down.
        track_pt = None
        track_shape = None
        return
 
    u, v = new_pts[0, 0]
    if u < 0 or u >= res[0] or v < 0 or v >= res[1]:
        # Point has left the visible frame -- clear track, ready for the next object.
        track_pt = None
        track_shape = None
        return
 
    track_pt = new_pts
 
 
def register_new_object(oid, shape, r, theta_obj):
    v = get_conveyor_vel()
    if abs(v) < 1e-6:
        return
    # NOTE: direction of increasing theta must match the belt's direction of travel.
    # If the arm gets triggered too early/late by roughly a full loop, flip the sign
    # of dtheta below (use (theta_obj - theta_eff) % (2*pi) instead).
    dtheta = (theta_eff - theta_obj) % (2 * math.pi)
    ds = dtheta * R_PATH
    t_arrival = sim.getSimulationTime() + ds / abs(v)
    scheduled.append({'id': oid, 'shape': shape, 'r': r, 't': t_arrival})
    wait = ds / abs(v)
    print(f"[detect] id={oid} shape={shape} r={r:.3f}m ds={ds:.3f}m "
          f"-> strike in {wait:.2f}s (t={t_arrival:.2f}s)")
    

# ---------------------------------------------------------------------------
# Arm execution
# ---------------------------------------------------------------------------
# Measure these empirically: time for the push trajectory from "ready" pose
# to the strike point and back. Can also depend on r (longer reach = longer time).
ARM_TRAVEL_TIME = {'cylinder': 0.6, 'cuboid': 0.6, 'star': 0.7}  # TUNE


def arm_control_step():
    now = sim.getSimulationTime()
    for job in list(scheduled):
        lead = ARM_TRAVEL_TIME.get(job['shape'], 0.6)
        if now >= job['t'] - lead:
            push_object(job['shape'], job['r'])
            scheduled.remove(job)


def push_object(shape, r):
    x = center_pos[0] + r * math.cos(theta_eff)
    y = center_pos[1] + r * math.sin(theta_eff)
    z = strike_pos[2]
    move_arm_to_push(x, y, z, shape)


def move_arm_to_push(x, y, z, shape):
    # Replace with your actual push motion: e.g. move to a pre-push pose,
    # sweep through (x, y, z) toward the bridge for `shape`, then retract.
    sim.setObjectPosition(ik_target, -1, [x, y, z])
    # TODO: step/wait until the arm reports it reached the target,
    # then move toward BRIDGE_TARGETS[shape] and return to ready pose.


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
sim.startSimulation()
time.sleep(0.5)
try:
    while True:
        detect_new_objects()
        arm_control_step()
        time.sleep(0.02)
except KeyboardInterrupt:
    sim.stopSimulation()