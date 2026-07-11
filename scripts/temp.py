import time
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import math

client = RemoteAPIClient()
sim = client.require('sim')

conveyorCenter = sim.getObject('/conveyorSystem/ConveyorCenter')
conveyorRadius = sim.getObject('/conveyorSystem/ConveyorRadius')

sim.startSimulation()
time.sleep(0.5)

center_pos = sim.getObjectPosition(conveyorCenter, -1)

# Pick any point that sits exactly on the belt's centerline path,
# e.g. the strike_dummy if it's placed on the belt, or a dedicated
# reference dummy placed on the belt center-line.
ref_pos = sim.getObjectPosition(conveyorRadius, -1)

R_path = math.hypot(ref_pos[0] - center_pos[0], ref_pos[1] - center_pos[1])
loop_length = 2 * math.pi * R_path

print(f"R_path = {R_path:.4f} m, loop_length = {loop_length:.4f} m")

sim.stopSimulation()