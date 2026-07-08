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

from sim_interface import CoppeliaSimController
from routine import RoutineLoader, RoutineExecutor


def main():
    routine_path = sys.argv[1] if len(sys.argv) > 1 else "routines/routine_00.json"

    print(f"Connecting to CoppeliaSim...")
    sim = CoppeliaSimController().connect()
    print("Connected.")

    print(f"Loading routine: {routine_path}")
    waypoints = RoutineLoader.load(routine_path)
    print(f"{len(waypoints)} waypoints loaded.")

    executor = RoutineExecutor(interface=sim)
    executor.run(waypoints)


if __name__ == "__main__":
    main()
