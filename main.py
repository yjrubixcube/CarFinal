import time
import random
import threading
import pygame
import sys
import os

from vehicle import Vehicle
from managers import *
from const import *


def create_map():
    pass

def main():

    
    

    # pygame init
    WIDTH, HEIGHT = 1080, 960
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("simulation")

    main_thread = threading.Thread(target=create_map, args=())
    main_thread.daemon = True
    main_thread.start()

    # create vehicles
    for i in range(N):
        v_type = random.choice(["HDV", "CAV"])
        v_lane = random.choice(range(4))
        v_target = random.choice(range(4, 8))
        physics = {
            POSITION: random.uniform(),
            VELOCITY: random.uniform(SPEED_LIMIT/2, SPEED_LIMIT),
            ACCEL: 0
        }
        Vehicle(i, v_type, v_lane, v_target, physics)


    # simulate
    while 1:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                sys.exit(0)
                

if __name__ == "__main__":
    main()