# -*- coding: utf-8 -*-
"""
find_capacity.py  —  find the demand ceiling of the network before the big batch.

Runs QAWP (ground truth, ray casting off, so it is fast) at increasing total
vehicle demand and reports delay, Level of Service, insertion backlog and
teleports for each level.

  * backlog  = vehicles that could not even ENTER the network because the
               approaches were full.  Once this grows, the intersection is
               saturated and delays measured beyond that point understate
               the real congestion (the worst-off vehicles never enter).
  * teleports = vehicles SUMO removed after being stuck 100 s (gridlock).

The suggested capacity is the highest demand with backlog below 2 % and no
teleports.  Use it to set the demand lists at the top of run_experiments.py.

Usage
    python find_capacity.py --jobs 4
    python find_capacity.py --start 2000 --stop 4500 --step 125 --seeds 3 --jobs 4
"""
import os, sys, csv, argparse, subprocess, statistics
from concurrent.futures import ThreadPoolExecutor

PY = sys.executable


def one(tv, seed):
    os.makedirs("runs_capacity", exist_ok=True)
    out = f"runs_capacity/cap_veh{tv}_seed{seed}.csv"
    if not os.path.exists(out):
        subprocess.run([PY, "traci_control_v3.py", "--strategy", "qawp",
                        "--total_veh", str(tv), "--cav_pct", "50", "--seed", str(seed),
                        "--ray_dt", "0", "--out", out],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        return tv, seed, list(csv.DictReader(open(out)))[0]
    except Exception:
        return tv, seed, None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--start", type=int, default=1000)
    ap.add_argument("--stop", type=int, default=5000)
    ap.add_argument("--step", type=int, default=250)
    ap.add_argument("--seeds", type=int, default=2)
    ap.add_argument("--jobs", type=int, default=1)
    a = ap.parse_args()
    seeds = [42, 123, 777, 2024, 999][:a.seeds]
    demands = list(range(a.start, a.stop + 1, a.step))
    tasks = [(tv, s) for tv in demands for s in seeds]
    print(f"Probing {len(demands)} demand levels x {len(seeds)} seeds = {len(tasks)} runs ...")
    with ThreadPoolExecutor(max_workers=max(1, a.jobs)) as pool:
        res = list(pool.map(lambda t: one(*t), tasks))

    rows = []
    print(f"\n{'demand':>7} {'veh delay':>10} {'LoS':>4} {'ped delay':>10} {'backlog %':>10} {'teleports':>10}")
    capacity = None
    for tv in demands:
        rs = [r for (d, s, r) in res if d == tv and r]
        if not rs:
            print(f"{tv:>7}   (failed)"); continue
        f = lambda k: statistics.mean(float(r[k]) for r in rs)
        bl = statistics.mean(100 * float(r["insertion_backlog"]) / max(1, float(r["loaded"])) for r in rs)
        tp = f("teleports"); vd = f("avg_veh_delay"); pd = f("avg_ped_delay")
        los = "A" if vd < 10 else "B" if vd < 20 else "C" if vd < 35 else "D" if vd < 55 else "E" if vd < 80 else "F"
        print(f"{tv:>7} {vd:>9.1f}s {los:>4} {pd:>9.1f}s {bl:>9.1f}% {tp:>10.1f}")
        rows.append({"total_veh": tv, "avg_veh_delay": round(vd, 2), "los": los,
                     "avg_ped_delay": round(pd, 2), "backlog_pct": round(bl, 2), "teleports": tp})
        if bl < 2.0 and tp == 0:
            capacity = tv
    with open("capacity_probe.csv", "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0].keys())); w.writeheader(); w.writerows(rows)
    print(f"\nSuggested capacity (backlog < 2 %, no teleports): {capacity} veh/h")
    print("Saved: capacity_probe.csv")


if __name__ == "__main__":
    os.environ.setdefault("SUMO_HOME", "/usr/local/lib/python3.12/dist-packages/sumo")
    main()
