import os, sys
from const import *

if "SUMO_HOME" in os.environ:
    sys.path.append(os.path.join(os.environ["SUMO_HOME"], "tools"))
else:
    sys.exit("no sumo")

from sumolib import checkBinary
import traci

sumo_binary = checkBinary("sumo")
# sumo_binary = checkBinary("sumo-gui")

sumo_cmd = [sumo_binary, '-c', "TODO"]

traci.start(sumo_cmd)

step = 0

while step < TOTAL_STEPS:
    traci.simulationStep()
    

traci.close(False)