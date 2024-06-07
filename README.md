# Introduction to Intelligent Vehicles Final Project Report
## How to run
requires `Gurobi`, `SUMO`
```python
python main.py
```
## File Structures
### Scripts
```
genxml.py: generate necessary xml files
manager_milp.py: MILP solver function
main.py: main driver code
const.py: some constants
```

### Data
```
data/template: used to generate other xmls
data/supplement: SUMO intermediate files used to generate net.xml
data/intersection.net.xml: Map for SUMO
data/intersection.rou.xml: Departure time for vehicles
data/sumoresult.xml: Simulation results
```