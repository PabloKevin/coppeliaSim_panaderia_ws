"""
main.py
=======
Entry point. Connects to CoppeliaSim, loads the cartesian routine from JSON,
and runs it through the jerk-limited trajectory pipeline.

Prerequisites:
    pip install coppeliasim-zmqremoteapi-client
    CoppeliaSim open, with the scene loaded and simulation running
    (the ZMQ remote API server is built into recent CoppeliaSim versions --
    no extra add-on needed for 4.x).

Usage:
    python main.py
    python main.py routines/pick_and_place.json
"""

import sys
import time
from sim_interface import CoppeliaSimController
from routine import RoutineLoader, RoutineExecutor


def main():
    print(f"Connecting to CoppeliaSim...")
    sim = CoppeliaSimController().connect()
    print("Connected.")

    executor = RoutineExecutor(interface=sim)

    """routine_path = sys.argv[1] if len(sys.argv) > 1 else "routines/push_homeback_Right.json"
    waypoints = RoutineLoader.load(routine_path)
    executor.run(waypoints)

    routine_path = sys.argv[1] if len(sys.argv) > 1 else "routines/push_Right.json"

    

    print(f"Loading routine: {routine_path}")
    waypoints = RoutineLoader.load(routine_path)
    print(f"{len(waypoints)} waypoints loaded.")

    pushTime = 0.0
    reps = 10
    for _ in range(reps):
        time.sleep(0.5)
        start = time.time()
        executor.run(waypoints)
        finish = time.time()
        pushTime += finish - start
    pushTime = pushTime / reps
    

    print(f"Routine completed in average {pushTime:.2f} seconds.")"""

    sim.sim.startSimulation()
    routine_path = sys.argv[1] if len(sys.argv) > 1 else "routines/pick_and_place_final.json"
    waypoints = RoutineLoader.load(routine_path)
    executor.run(waypoints)


if __name__ == "__main__":
    main()
