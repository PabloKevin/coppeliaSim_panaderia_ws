import sys
import tty
import termios
import time
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

# --- Function to read single keypresses in Linux Terminal without hitting Enter ---
def get_key():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
        # Arrow keys in terminal return an escape sequence: \x1b[A, \x1b[B, etc.
        if ch == '\x1b':
            ch2 = sys.stdin.read(1)
            if ch2 == '[':
                ch3 = sys.stdin.read(1)
                if ch3 == 'A': return 'up'
                if ch3 == 'B': return 'down'
                if ch3 == 'C': return 'right'
                if ch3 == 'D': return 'left'
        elif ch.lower() == 'q':
            return 'quit'
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return None

# --- Initialize CoppeliaSim Connection ---
client = RemoteAPIClient()
sim = client.require('sim')

# Fetch joint handles matching your active setup
joint_configs = [
    {"name": "/base", "handle": sim.getObject('/EZZYbotARM_Mk2/motor1'), "step": 0.05, "pos": 0.0},
    {"name": "/shoulder", "handle": sim.getObject('/EZZYbotARM_Mk2/motor2'), "step": 0.05, "pos": 0.0},
    {"name": "/elbow", "handle": sim.getObject('/EZZYbotARM_Mk2/motor3'), "step": 0.05, "pos": 0.0},
    {"name": "/gripper", "handle": sim.getObject('/EZZYbotARM_Mk2/motor4'), "step": 0.05, "pos": 0.0},
]

limits = {
        'motor1': (-3.1415, 3.1415),
        'motor2': (-1.550, 0.950),
        'motor3': (-0.450, 1.350),
        'motor4': (0.0, 1.450) #0.800 has more sense if gripper is boolean

    }

print("Starting simulation...")
sim.startSimulation()
time.sleep(0.5) # Let physics initialize

# Read live initial joint values directly from CoppeliaSim scene
for joint in joint_configs:
    joint["pos"] = sim.getJointPosition(joint["handle"])

current_idx = 0

print("\n" + "="*50)
print(" EZZYbotARM Mk2 - Keyboard Teleop Controller")
print("="*50)
print("  UP / DOWN Arrows  : Switch selected joint")
print("  LEFT / RIGHT Arrows: Decrease / Increase angle")
print("  Press 'q'         : Quit and Stop Simulation")
print("="*50 + "\n")

try:
    while True:
        # Format a clean string displaying the active joint selection
        active_name = joint_configs[current_idx]["name"]
        active_pos = joint_configs[current_idx]["pos"]
        
        # \r overwrites the line in place so the terminal doesn't fill up with spam
        sys.stdout.write(f"\r[Active Joint] -> {active_name:<15} | Target Value: {active_pos:6.3f} rad  ")
        sys.stdout.flush()

        # Catch terminal key stroke
        key = get_key()

        if key == 'quit':
            print("\nExiting interactive mode...")
            break
            
        elif key == 'up':
            # Cycle backward through the joints list
            current_idx = (current_idx - 1) % len(joint_configs)
            
        elif key == 'down':
            # Cycle forward through the joints list
            current_idx = (current_idx + 1) % len(joint_configs)
            
        elif key == 'right':
            # Increase target positions
            joint_configs[current_idx]["pos"] += joint_configs[current_idx]["step"]
            sim.setJointTargetPosition(joint_configs[current_idx]["handle"], joint_configs[current_idx]["pos"])
            
        elif key == 'left':
            # Decrease target positions
            joint_configs[current_idx]["pos"] -= joint_configs[current_idx]["step"]
            sim.setJointTargetPosition(joint_configs[current_idx]["handle"], joint_configs[current_idx]["pos"])

except Exception as e:
    print(f"\nAn error occurred: {e}")

finally:
    print("Stopping simulation process.")
    sim.stopSimulation()