# Simulation results

| File | Content |
|---|---|
| `A-D_demand_mingreen_interval_weights.csv` | A_stress (demand sweep), B_mingreen (minimum green), C_decisiondt (decision interval), D_rq2 (queue weight and pedestrian volume) |
| `E_penetration_20-100.csv` | E_penetration_cav and E_penetration_gt: CAV perception against perfect information, 20–100 % CAVs |
| `G-J_vru_weight_interval_grid_zone.csv` | G_vruweight (VRU class weight), H_dt (decision interval), I_grid (penetration × queue weight), J_zone (roadside VRU sensor, outlook only) |
| `K-L_vru_multiplier_penetration_5-20.csv` | K_vrumult (VRU weight multipliers), L_pen_cav / L_pen_gt / L_pen_fixed (5–20 % CAVs) |
| `green_phase_log.csv`, `green_phase_summary.csv` | Length of every green phase in the logging runs |
| `capacity_probe.csv` | Capacity probe of the test intersection |

**Note on versions:** `A-D_demand_mingreen_interval_weights.csv` also contains rows of the experiments E_penetration_cav and E_penetration_gt. These come from an earlier version of the perception model and are superseded. The thesis uses `E_penetration_20-100.csv` for all perception results.
