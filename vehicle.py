import random
from const import *
class Vehicle:
    def __init__(self, id: int, type: str, lane: int, target: int, physics: dict) -> None:
        self.id = id
        self.lane = lane
        assert (type == "CAV" or type == "HDV")
        # pos, vel, accel
        self.physics = physics

        self.type = type
        random.seed(id)

        self.color = (random.randint(0, 255), random.randint(0, 255), random.randint(0, 255))