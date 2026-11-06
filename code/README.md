# Simulation code

Code of the master's thesis "Queue-Aware Delay Balancing at Automated Intersections under Mixed Traffic" (Technical University of Munich, 2026).

Requirements: SUMO 1.27 with its Python tools (TraCI, sumolib), Python 3.

| File | Content |
|---|---|
| `qawp_common.py` | Shared functions: signal model, phase state, class weighting, observability |
| `traci_control_v3.py` | QAWP and the fixed-time baselines with perfect information |
| `traci_control_v4.py` | QAWP with CAV perception |
| `ray_casting_v2.py` | Ray-casting sensor model of the CAVs |
| `run_experiments.py` | Experiments A–E |
| `run_oct.py` | Experiments G–J |
| `run_oct2.py` | Experiments K–L |
| `run_green_log.py` | Logging run for the length of every green phase |
| `find_capacity.py` | Capacity of the test intersection |
| `network2.net.xml`, `demand.rou.xml`, `run_simulation.sumocfg` | Network, demand and SUMO configuration |

Example (one simulation run with the SUMO window):

```
python traci_control_v3.py --strategy qawp --total_veh 2500 --cav_pct 50 --ped_vol 300 --seed 42 --gui
```
