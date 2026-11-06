# -*- coding: utf-8 -*-
"""
run_green_log.py  -  logs every green interval QAWP produces, to show HOW the
controller behaves: how long each green lasts, and whether QAWP switched at the
first moment it was allowed to (green length == minimum green) or kept the green.

Ground truth, default settings (w1 = w2 = 0.5, decision interval 5 s).
Writes results/green_log.csv  (one row per green interval)
and   results/green_summary.csv (one row per run).

Usage:  python run_green_log.py
"""
import os, sys, csv
os.environ.setdefault("SUMO_HOME", "/usr/local/lib/python3.12/dist-packages/sumo")
sys.argv = [sys.argv[0]]
import traci, qawp_common as C, traci_control_v3 as V

def one(total_veh, min_green, seed, dt=5):
    df, _ = C.make_demand_with_split("demand.rou.xml", total_veh, 50)
    m = C.SignalModel(C.net_file_from_cfg())
    traci.start(["sumo", "-c", "run_simulation.sumocfg", "--route-files", df, "--seed", str(seed),
                 "--time-to-teleport", "100", "--no-warnings", "true", "--no-step-log", "true"])
    ctrl = V.QAWPController(m, C.SignalActuator(traci, m), V.GroundTruthPerception(m), 0.5, 0.5, min_green, dt)
    ctrl.reset()
    greens = []; decisions = kept = 0
    orig = ctrl.act.begin
    def begin(target):
        greens.append((ctrl.act.cur, ctrl.act.green_age))
        orig(target)
    ctrl.act.begin = begin
    for _ in range(3600):
        due = (ctrl.act.in_green and ctrl.act.green_age + 1 >= min_green
               and (ctrl.act.green_age + 1) % dt == 0)
        before = ctrl.switches
        traci.simulationStep(); ctrl.step()
        if due:
            decisions += 1
            kept += (ctrl.switches == before)
    traci.close(); os.unlink(df)
    return greens, decisions, kept

if __name__ == "__main__":
    os.makedirs("results", exist_ok=True)
    rows, summ = [], []
    for mg in (10, 20):
        for tv in (1500, 2500, 3250):
            for sd in (42, 123, 777):
                g, dec, kept = one(tv, mg, sd)
                for ph, dur in g:
                    rows.append(dict(min_green=mg, total_veh=tv, seed=sd, phase=ph, green_s=dur,
                                     at_first_opportunity=int(dur == mg)))
                share = 100 * sum(1 for _, d in g if d == mg) / len(g)
                summ.append(dict(min_green=mg, total_veh=tv, seed=sd, greens=len(g), decisions=dec,
                                 kept=kept, pct_switched_at_first_opportunity=round(share, 1)))
                print(f"mg {mg:>2}  {tv} veh/h  seed {sd:<4}: {len(g)} greens, "
                      f"{share:.0f}% ended at the first opportunity")
    for fn, data in (("results/green_log.csv", rows), ("results/green_summary.csv", summ)):
        with open(fn, "w", newline="") as fh:
            w = csv.DictWriter(fh, fieldnames=list(data[0].keys())); w.writeheader(); w.writerows(data)
    print("saved results/green_log.csv and results/green_summary.csv")
