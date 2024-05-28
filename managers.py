from const import *
import random
from vehicle import Vehicle
class Manager:
    def __init__(self, incoming_vehicles: list[Vehicle]) -> None:
        self.incoming_vehicles = incoming_vehicles

class FCFS(Manager):
    def __init__(self, incoming_vehicles: list[Vehicle]) -> None:
        super().__init__(incoming_vehicles)
        
        """
        {(a, b, c, d)}
        v1: a to b
        v2: c to d
        v1, v2 conflict
        """
        self.traj_conflict = set()

    def solve():
        # change incoming vehicles priority
        pass