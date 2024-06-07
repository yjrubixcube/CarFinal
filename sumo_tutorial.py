import os, sys
from const import *
import manager_milp

if "SUMO_HOME" in os.environ:
    sys.path.append(os.path.join(os.environ["SUMO_HOME"], "tools"))
else:
    sys.exit("no sumo")

from sumolib import checkBinary
import traci

# sumo_binary = checkBinary("sumo")
sumo_binary = checkBinary("sumo-gui")

sumo_cmd = [sumo_binary, '-c', "intersection.sumocfg"]

traci.start(sumo_cmd)
# TODO
# https://sumo.dlr.de/docs/TraCI/Interfacing_TraCI_from_Python.html#adding_a_steplistener

# first_time = {}
in_range = {}

saved_milp_data = {}

in_intersection = {}


def in_manager_range(data):
    # in_edge_ids = {"iN":0, "iE":0, "iW":0, "iS":0}
    changed = False
    count = 0
    # print(data)
    for vid in data:
        position = data[vid][traci.constants.VAR_POSITION]
        lane = data[vid][traci.constants.VAR_LANE_ID]
        # in_edge, out_edge = data[vid][traci.constants.VAR_EDGES]
        # in_edge_ids[in_edge] += 1
        x, y = position
        # if "intersection" in lane:# and "o" in lane:
        #     if vid in in_range:
        #         in_range.pop(vid)
        #     if vid not in in_intersection:
        #         in_intersection[vid] = data[vid]
        #         changed = True
        #         print(f"{vid} entered intersection")
        #     # ct = traci.simulation.getTime()
            
        # elif "i" in lane:
        if "i" in lane:
            # if vid in in_range:
            #     count += 1
            # elif vid not in in_range:
                changed = True
                if "N" in lane:
                    if y < CENTER_Y + IM_RANGE:
                        in_range[vid] = data[vid]
                        count += 1
                        # print(f"{vid} on lane {lane} entered manager range")
                elif "E" in lane:
                    if x < CENTER_X + IM_RANGE:
                        in_range[vid] = data[vid]
                        count += 1
                        # print(f"{vid} on lane {lane} entered manager range")
                elif "W" in lane:
                    if x > CENTER_X - IM_RANGE:
                        in_range[vid] = data[vid]
                        count += 1
                        # print(f"{vid} on lane {lane} entered manager range")
                elif "S" in lane:
                    if y > CENTER_Y - IM_RANGE:
                        in_range[vid] = data[vid]
                        count += 1
                        # print(f"{vid} on lane {lane} entered manager range")
        else:
            # out lane
            # if vid in in_range:
            #     in_range.pop(vid)
            #     print(f"{vid} left manager range")
            if vid in in_range:
                in_range.pop(vid)
                print(f"{vid} left intersection range")
    # print(count, len(in_range))
    # assert count == len(in_range)
    return changed
    # if last_in_range_len != len(in_range):
    #     last_in_range_len = len(in_range)
    # return in_intersection, in_range

def get_no_leader_time(cur_speed, max_decel, dist):
    if dist <= IN_INTERSECTION:
        return 0
    # if cur_speed < SPEED_LIMIT / 3:
    #     return 1
    return dist / cur_speed
    decel_time = cur_speed / max_decel
    # v**2 = v0 ** 2 + 2aX
    decel_dist = cur_speed ** 2 / (2 * max_decel)
    if decel_dist > dist:
        return decel_time
    
    dist -= decel_dist
    constant_speed_time = dist / SPEED_LIMIT
    # constant_speed_time = dist / cur_speed
    return decel_time + constant_speed_time

def get_with_leader_time(vid, cur_speed, max_decel, dist, leader):
    return get_no_leader_time(cur_speed, max_decel, dist)
    leader, leader_dist = leader
    follow_speed = traci.vehicle.getFollowSpeed(vid, cur_speed, MIN_GAP, traci.vehicle.getSpeed(leader), traci.vehicle.getDecel(leader), leader)

    decel_time = follow_speed / max_decel
    # v**2 = v0 ** 2 + 2aX
    decel_dist = follow_speed ** 2 / (2 * max_decel)
    if decel_dist > dist:
        return decel_time
    
    dist -= decel_dist
    constant_speed_time = dist / SPEED_LIMIT
    return decel_time + constant_speed_time

def lane2index(lane):
    if "N" in lane:
        return 1
    if "E" in lane:
        return 0
    if "W" in lane:
        return 2
    if "S" in lane:
        return 3
    raise ZeroDivisionError

def estimate_time_to_intersection(data):
    milp_data = [[[], [], [], []], [[], [], [], []], [[], [], [], []]]
    for vid in data:

        lane = data[vid][traci.constants.VAR_LANE_ID]
        lane_index = lane2index(lane)
    
        if vid in saved_milp_data:
            # A
            milp_data[0][lane_index].append(saved_milp_data[vid][0])
            # H
            milp_data[1][lane_index].append(saved_milp_data[vid][1])
            # target
            milp_data[2][lane_index].append(saved_milp_data[vid][2])
            continue

        in_edge, out_edge = data[vid][traci.constants.VAR_EDGES]
        x, y = data[vid][traci.constants.VAR_POSITION]
        leader = traci.vehicle.getLeader(vid)
        speed = traci.vehicle.getSpeed(vid)
        decel = traci.vehicle.getDecel(vid)
    

        if "N" in lane:
            dist = y - CENTER_Y
            if leader is None:
                # pure physics
                estimated_time = get_no_leader_time(speed, decel, dist)
            else:
                # use leader physics
                estimated_time = get_with_leader_time(vid, speed, decel, dist, leader)
        elif "E" in lane:
            dist = x - CENTER_X
            if leader is None:
                # pure physics
                estimated_time = get_no_leader_time(speed, decel, dist)
            else:
                # use leader physics
                estimated_time = get_with_leader_time(vid, speed, decel, dist, leader)
        elif "W" in lane:
            dist = CENTER_X - x
            if leader is None:
                # pure physics
                estimated_time = get_no_leader_time(speed, decel, dist)
            else:
                # use leader physics
                estimated_time = get_with_leader_time(vid, speed, decel, dist, leader)
        elif "S" in lane:
            dist = CENTER_Y - y
            if leader is None:
                # pure physics
                estimated_time = get_no_leader_time(speed, decel, dist)
            else:
                # use leader physics
                estimated_time = get_with_leader_time(vid, speed, decel, dist, leader)
        
        if "N" in out_edge:
            target_index = 1
        elif "E" in out_edge:
            target_index = 0
        elif "W" in out_edge:
            target_index = 2
        elif "S" in out_edge:
            target_index = 3
        ct = traci.simulation.getTime()
        v_type = traci.vehicle.getTypeID(vid)
        saved_milp_data[vid] = [estimated_time + ct, v_type == "HDV", target_index]
        # A
        milp_data[0][lane_index].append(saved_milp_data[vid][0])
        # H
        milp_data[1][lane_index].append(saved_milp_data[vid][1])
        # target
        milp_data[2][lane_index].append(saved_milp_data[vid][2])
        # print(flush=True)
        # print(milp_data)
    print("est time")
    print(milp_data)
    milp_sol = manager_milp.solve(milp_data[0], milp_data[1], milp_data[2], 1, 3)
    # milp_sol = manager_milp.fast_solve(milp_data[0], milp_data[1], milp_data[2], 5, 1, 3)
    for i in range(len(milp_sol)):
        for j in range(len(milp_sol[i])):
            milp_sol[i][j] = int(milp_sol[i][j] + 1)
    return milp_sol
slowing = {}

def change_behaviour(data, manager_time):
    vid2index = {}
    minind = [0, 0, 0, 0]
    ct = traci.simulation.getTime()
    # print(data)
    # print(manager_time)
    for vid in data:
        in_edge, out_edge = data[vid][traci.constants.VAR_EDGES]
        x, y = data[vid][traci.constants.VAR_POSITION]
        lane_index = lane2index(in_edge)
        
        if "N" in in_edge:
            dist = y - CENTER_Y
        elif "E" in in_edge:
            dist = x - CENTER_X
        elif "W" in in_edge:
            dist = CENTER_X - x
        elif "S" in in_edge:
            dist = CENTER_Y - y
        print("dist", dist, vid)

        vid2index[vid] = minind[lane_index]
        minind[lane_index] += 1
        return
        if dist < SLOWDOWN_RANGE:
            line = manager_time[lane_index]
            # print(vid, vid2index[vid])
            # curspeed = traci.vehicle.getSpeed(vid)
            if line[vid2index[vid]] > ct :#- curspeed/MAX_DECEL:
                # not your time
                # traci.vehicle.setSpeed(vid, 0)
                oldspeed = traci.vehicle.getSpeed(vid)
                if vid not in slowing:
                    traci.vehicle.slowDown(vid, 0, -SPEED_LIMIT/MAX_DECEL)
                    slowing[vid] = -SPEED_LIMIT/MAX_DECEL
                else:
                    slowing[vid] -= 1
                    if slowing[vid] < 1:
                        slowing[vid] = 1
                        traci.vehicle.setSpeed(vid, 0)
                    else:
                        traci.vehicle.slowDown(vid, 0, slowing[vid])
                if dist < IN_INTERSECTION:
                    traci.vehicle.setSpeed(vid, 0)
                traci.vehicle.setSpeedMode(vid, 0)

                print(f"slow down {vid} from {oldspeed} to {traci.vehicle.getSpeed(vid)}")
                # exit()
            else:
                if vid in slowing:
                    slowing.pop(vid)
                # traci.vehicle.setAcceleration(vid, MAX_ACCEL, SPEED_LIMIT/MAX_ACCEL)
                traci.vehicle.setSpeed(vid, -1)
                traci.vehicle.setSpeedMode(vid, 0)

                print(f"speed up {vid}")
                # exit()

# to MILP: estimated time of arrival to intersection
# from MILP: when to let vehicle pass
flag = True

while traci.simulation.getMinExpectedNumber():
    for vid in traci.simulation.getDepartedIDList():
        traci.vehicle.subscribe(vid, [traci.constants.VAR_POSITION, traci.constants.VAR_LANE_ID, traci.constants.VAR_EDGES])

    data = traci.vehicle.getAllSubscriptionResults()
    changed = in_manager_range(data)
    # print("data")
    # print(data)
    # in_intersection, in_range = in_manager_range(data)
    print("intersection")
    print(in_intersection)
    print("in rnage")
    print(in_range)
    print("cur time")
    print(traci.simulation.getTime())
    if in_range:
        if changed:
            manager_time = estimate_time_to_intersection(in_range)
            # print(manager_time)
            print("manager time")
            print(manager_time)
        change_behaviour(in_range, manager_time)
    if len(in_range) == 0 and len(in_intersection) == 0:
        print(traci.simulation.getTime())

    traci.simulationStep()

traci.close(False)

'''
16
11.5
7
2.5
0
'''