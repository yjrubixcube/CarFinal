import random

NUM_LANES = 4
VEHICLES_PER_LANE = 5
HV_RATIO = 0.5
TOTAL_VEHICLES = NUM_LANES * VEHICLES_PER_LANE

TOTAL_HDVS = int(TOTAL_VEHICLES * HV_RATIO)
TOTAL_CAVS = TOTAL_VEHICLES - TOTAL_HDVS
hdv_lane = ["HN_NUM", "HE_NUM", "HW_NUM", "HS_NUM"]
cav_lane = ["CN_NUM", "CE_NUM", "CW_NUM", "CS_NUM"]
data_dict = {
    "CAV_PROB": 0.5,
    "HDV_PROB": 0.5,
    "CN_NUM": 0,
    "CE_NUM": 0,
    "CW_NUM": 0,
    "CS_NUM": 0,
    "HN_NUM": 0,
    "HE_NUM": 0,
    "HW_NUM": 0,
    "HS_NUM": 0,   
}

for i in range(TOTAL_HDVS):
    ind = random.randint(0, 3)
    data_dict[hdv_lane[ind]] += 1


for i in range(TOTAL_CAVS):
    ind = random.randint(0, 3)
    data_dict[cav_lane[ind]] += 1

print(data_dict)

route_file_name = "data/supplement/intersection.tmp.rou.xml"
route_template_name = "data/template/rou.xml"

with open(route_template_name, 'r') as route_template_file:
    route_template = route_template_file.read()

for s in data_dict:
    route_template = route_template.replace(s, str(data_dict[s]))

with open(route_file_name, "w") as file:
    print(route_template, file=file)
# exit()
import subprocess

subprocess.run("netconvert --node-files=data/supplement/intersection.nod.xml --edge-files=data/supplement/intersection.edg.xml --connection-files=data/supplement/intersection.con.xml --output-file=data/intersection.net.xml")
res = subprocess.check_output(f"jtrrouter -n data/intersection.net.xml -r {route_file_name} -t data/supplement/intersection.turns.xml -o data/intersection.rou.xml")

res = res.decode()
if "Success" in res:
    s = res.split("\r")
    # print(s)
    for line in s[::-1]:
        if "Reading up to time step" in line:
            t = float(line[25:])
            print(t)
            break

sumocfg_data = {
    "END_VALUE": t
}
cfg_template_name = "data/template/sumocfg.xml"
cfg_file_name = "intersection.sumocfg"

with open(cfg_template_name, 'r') as sumocfg_template_file:
    sumocfg_template = sumocfg_template_file.read()

for s in sumocfg_data:
    sumocfg_template = sumocfg_template.replace(s, str(sumocfg_data[s]))

with open(cfg_file_name, 'w') as file:
    print(sumocfg_template, file=file)