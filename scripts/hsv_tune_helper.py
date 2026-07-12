"""
Interactive HSV range tuner for the cookie color (e.g. brown).
Move the trackbars until the "mask" window shows ONLY the cookies as white,
with the belt/arm/background as black. Read off the final H/S/V min/max
values printed on quit and paste them into BROWN_LOW / BROWN_HIGH in the
main control script.

Press 'q' or ESC to quit and print the final values.
"""

import numpy as np
import cv2
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

client = RemoteAPIClient()
sim = client.require('sim')

vision_sensor = sim.getObject('/visionSensor')  # TUNE if your path differs


def nothing(x):
    pass


def grab_frame():
    img, res = sim.getVisionSensorImg(vision_sensor)   # NOTE: (image, resolution) order
    img = np.frombuffer(img, dtype=np.uint8).reshape(res[1], res[0], 3)
    img = cv2.flip(img, 0)
    return cv2.cvtColor(img, cv2.COLOR_RGB2BGR)


def main():
    sim_was_running = sim.getSimulationState() != sim.simulation_stopped
    if not sim_was_running:
        sim.startSimulation()
        import time
        time.sleep(0.3)

    cv2.namedWindow('controls')
    cv2.createTrackbar('H min', 'controls', 0, 179, nothing)
    cv2.createTrackbar('H max', 'controls', 179, 179, nothing)
    cv2.createTrackbar('S min', 'controls', 0, 255, nothing)
    cv2.createTrackbar('S max', 'controls', 255, 255, nothing)
    cv2.createTrackbar('V min', 'controls', 0, 255, nothing)
    cv2.createTrackbar('V max', 'controls', 255, 255, nothing)

    while True:
        frame = grab_frame()
        # Scale up for visibility -- your sensor is only 128x128
        display_frame = cv2.resize(frame, (512, 512), interpolation=cv2.INTER_NEAREST)
        hsv = cv2.cvtColor(display_frame, cv2.COLOR_BGR2HSV)

        h_min = cv2.getTrackbarPos('H min', 'controls')
        h_max = cv2.getTrackbarPos('H max', 'controls')
        s_min = cv2.getTrackbarPos('S min', 'controls')
        s_max = cv2.getTrackbarPos('S max', 'controls')
        v_min = cv2.getTrackbarPos('V min', 'controls')
        v_max = cv2.getTrackbarPos('V max', 'controls')

        lower = np.array([h_min, s_min, v_min])
        upper = np.array([h_max, s_max, v_max])
        mask = cv2.inRange(hsv, lower, upper)
        result = cv2.bitwise_and(display_frame, display_frame, mask=mask)

        cv2.imshow('original', display_frame)
        cv2.imshow('mask', mask)
        cv2.imshow('filtered', result)

        key = cv2.waitKey(30) & 0xFF   # REQUIRED for windows to actually refresh
        if key == ord('q') or key == 27:
            break

    cv2.destroyAllWindows()
    if not sim_was_running:
        sim.stopSimulation()

    print("\nFinal HSV range (paste into main script):")
    print(f"BROWN_LOW  = np.array([{h_min}, {s_min}, {v_min}])")
    print(f"BROWN_HIGH = np.array([{h_max}, {s_max}, {v_max}])")


if __name__ == '__main__':
    main()
