"""
Calibration helper: grabs one frame from the vision sensor, displays it,
and prints the pixel (u, v) of every point you click. Use this to fill in
IMG_PTS for the homography calibration in bakery_conveyor_control.py.

Click on each of your 4 calibration markers, in the SAME ORDER you listed
their handles in `calib_markers` in the main script. Press 'r' to grab a
fresh frame, 'q' or ESC to quit.
"""

import numpy as np
import cv2
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

client = RemoteAPIClient()
sim = client.require('sim')

vision_sensor = sim.getObject('/visionSensor')  # TUNE if your path differs

clicked_points = []


def grab_frame():
    img, res = sim.getVisionSensorImg(vision_sensor)   # NOTE: (image, resolution) order
    img = np.frombuffer(img, dtype=np.uint8).reshape(res[1], res[0], 3)
    img = cv2.flip(img, 0)                      # CoppeliaSim images are bottom-up
    img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)   # OpenCV expects BGR
    return img


def on_mouse(event, u, v, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        clicked_points.append((u, v))
        print(f"Clicked pixel #{len(clicked_points)}: (u={u}, v={v})")


def main():
    sim_was_running = sim.getSimulationState() != sim.simulation_stopped
    if not sim_was_running:
        sim.startSimulation()
        import time
        time.sleep(0.3)  # let the sim settle before grabbing the first frame

    cv2.namedWindow('VisionSensor - click calibration points')
    cv2.setMouseCallback('VisionSensor - click calibration points', on_mouse)

    frame = grab_frame()

    print("Click each calibration marker in order. Press 'r' to refresh the "
          "frame, 'q'/ESC to quit and print the final list.")

    while True:
        display = frame.copy()
        for i, (u, v) in enumerate(clicked_points):
            cv2.circle(display, (u, v), 5, (0, 0, 255), -1)
            cv2.putText(display, str(i), (u + 8, v - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

        cv2.imshow('VisionSensor - click calibration points', display)
        key = cv2.waitKey(30) & 0xFF

        if key == ord('r'):
            frame = grab_frame()
            clicked_points.clear()
            print("Frame refreshed, click list cleared.")
        elif key == ord('q') or key == 27:  # ESC
            break

    cv2.destroyAllWindows()

    if not sim_was_running:
        sim.stopSimulation()

    print("\nFinal IMG_PTS array (copy into bakery_conveyor_control.py):\n")
    print("IMG_PTS = np.array([")
    for (u, v) in clicked_points:
        print(f"    [{u}, {v}],")
    print("], dtype=np.float32)")


if __name__ == '__main__':
    main()