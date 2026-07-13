"""
CoppeliaSim Object Spawner / Generator
---------------------------------------
Duplicates three EXISTING template objects already present in the
CoppeliaSim scene (keeping their exact mesh, mass, inertia, and
dynamic properties), placing each copy at a controlled position with
a controlled time interval between spawns.

Requirements:
    pip install coppeliasim-zmqremoteapi-client

Before running:
    1. Make sure your three template objects already exist in the
       scene, fully configured (mass, dynamics, shape, etc.).
    2. Give them unique names, e.g. TemplatePartA, TemplatePartB,
       TemplatePartC (edit TEMPLATE_NAMES below to match).
    3. Optionally move the templates out of your working area (e.g.
       far below the floor) so only the spawned copies are "active"
       in the workspace. If you'd rather keep templates non-dynamic
       and only make the *copies* dynamic, see the note in
       spawn_object() below.

Usage:
    1. Open CoppeliaSim with your scene loaded.
    2. Run: python3 object_spawner.py
"""

import time
import itertools
from coppeliasim_zmqremoteapi_client import RemoteAPIClient


# Names (scene tree paths) of your existing template objects.
# Use the object's path as shown in the scene hierarchy, e.g. "/Gear"
# or just its name if it's at the top level, e.g. "Gear".
TEMPLATE_NAMES = ["/Galleta_Cuadrada", "/Galleta_Redonda", "/Galleta_Estrella"]

# How many objects to spawn in total (set to None for infinite)
MAX_OBJECTS = 9

# Minimum allowed distance (meters) to any existing "Galleta" object
# before a new one is allowed to spawn at a given position
MIN_SPAWN_DISTANCE = 0.3
# ---------------------------------------------------------------------
 
 
def connect():
    client = RemoteAPIClient()
    sim = client.require('sim')
    return sim
 
 
def get_objects_by_prefix(sim, prefix):
    """
    Return a list of (handle, alias, position) for every object in the
    scene whose name/alias starts with `prefix`.
    """
    all_handles = sim.getObjectsInTree(sim.handle_scene, sim.handle_all, 0)
 
    matches = []
    for h in all_handles:
        alias = sim.getObjectAlias(h)
        if alias.startswith(prefix):
            pos = sim.getObjectPosition(h, -1)  # world frame
            matches.append((h, alias, pos))
 
    return matches
 
 
def is_position_free(sim, candidate_pos, prefix="Galleta", min_distance=0.1):
    """
    Check that no existing object whose name starts with `prefix` is
    within `min_distance` meters of candidate_pos (x, y, z).
    Returns True if the spot is free, False if a "Galleta" is too close.
    """
    existing = get_objects_by_prefix(sim, prefix)
 
    for handle, alias, pos in existing:
        dx = pos[0] - candidate_pos[0]
        dy = pos[1] - candidate_pos[1]
        dz = pos[2] - candidate_pos[2]
        dist = (dx ** 2 + dy ** 2 + dz ** 2) ** 0.5
 
        if dist < min_distance:
            print(f"  -> Blocked: {alias} is {dist:.3f} m away "
                  f"(min allowed {min_distance} m)")
            return False
 
    return True
 
 
def get_template_handles(sim):
    """Look up the handles of the three template objects by name."""
    handles = []
    for name in TEMPLATE_NAMES:
        h = sim.getObject(name)
        handles.append(h)
    return handles
 
 
def spawn_object(sim, template_handle, position, index):
    """
    Duplicate a template object (mesh, mass, dynamics, everything)
    and place the copy at the given position.
    """
    # copyPasteObjects duplicates the object (and any children/scripts)
    # exactly, including physical properties. Returns a list of new
    # handles (one per input handle, plus any children).
    new_handles = sim.copyPasteObjects([template_handle], 0)
    new_handle = new_handles[0]
 
    # Give the copy a readable, unique name
    orig_alias = sim.getObjectAlias(template_handle)
    sim.setObjectAlias(new_handle, f"{orig_alias}_copy{index}")
 
    # Place it in the world frame (-1 = absolute/world)
    sim.setObjectPosition(new_handle, -1, position)
 
    # If the copy is dynamic, reset its velocity/forces so it doesn't
    # inherit any leftover motion from the template
    try:
        sim.resetDynamicObject(new_handle)
    except Exception:
        pass  # not a dynamic object, safe to ignore
 
    print(f"[{time.strftime('%H:%M:%S')}] Spawned {orig_alias}_copy{index} "
          f"at {position}")
 
    return new_handle
 
 
def main():
    sim = connect()
    print("Connected to CoppeliaSim.")
 
    template_handles = get_template_handles(sim)
    print(f"Found {len(template_handles)} template objects.")
 
    # Start simulation if it isn't already running
    if sim.getSimulationState() == sim.simulation_stopped:
        sim.startSimulation()
 
    template_cycle = itertools.cycle(template_handles)
 
    count = 0
    try:
        while MAX_OBJECTS is None or count < MAX_OBJECTS:
            template = next(template_cycle)
 
            print(f"Checking position {SPAWN_POSITION} ...")
            if is_position_free(sim, SPAWN_POSITION, prefix="Galleta",
                                 min_distance=MIN_SPAWN_DISTANCE):
                spawn_object(sim, template, SPAWN_POSITION, count)
                count += 1
            else:
                print("  -> Skipping spawn, position occupied.")
 
            time.sleep(SPAWN_INTERVAL)
 
    except KeyboardInterrupt:
        print("\nStopped by user.")
 
    print(f"Done. Spawned {count} objects total.")
 
 
if __name__ == "__main__":
    main()
 