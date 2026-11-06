# -*- coding: utf-8 -*-
"""
run_oct2.py  —  second October batch (Intersection 2 only).

  K_vrumult     VRU weighting expressed as "a VRU counts m times a car":
                m = 1.5, 2, 3  ->  v = m/(1+m) = 0.6, 0.667, 0.75.
                Fills the gap between the earlier runs (v = 0.5, 0.7, 0.9).
                Ground truth, fast.

  L_pen         Penetration sweep to pin down the threshold at which
                CAV-perception QAWP becomes respectable:
                CAV share 5, 7, 10, 12, 15, 17, 20 %
                x demand 1000 / 1500 / 2500 / 3250 x 5 seeds, each run three ways:
                    gt         QAWP with perfect information   (fast)
                    fixedtime  the design signal programme     (fast)
                    cav        QAWP with CAV perception        (slow)
                Same seed = same traffic, so the three are compared per seed.

Results:  runs_oct2/  ->  results/master_oct2.csv  (+ .xlsx)
Relaunching resumes: finished runs are skipped.

Usage
    python run_oct2.py --dry_run
    python run_oct2.py --only K_vrumult L_pen_gt L_pen_fixed --jobs 4     # fast part
    python run_oct2.py --only L_pen_cav --jobs 4                          # slow part
    python run_oct2.py --jobs 4                                           # everything
"""
import os, sys, csv, time, argparse, subprocess
from collections import Counter
from concurrent.futures import ThreadPoolExecutor, as_completed

PY = sys.executable
RUNS_DIR = "runs_oct2"
RES_DIR = "results"
MASTER = os.path.join(RES_DIR, "master_oct2.csv")

SEEDS10 = [42, 123, 777, 2024, 999, 31337, 8675309, 271828, 161803, 112358]
SEEDS5 = SEEDS10[:5]

ZONE_R = 30          # m, radius of the roadside VRU sensor


def cfg(exp, script="v3", mode=None, total_veh=2500, cav_pct=50, ped_vol=None,
        w1=0.5, vru_w=0.5, zone=0, min_green=10, decision_dt=5, seed=42,
        strategy="qawp", detect_r=0):
    return dict(exp=exp, script=script, mode=mode, total_veh=total_veh,
                strategy=strategy, detect_r=detect_r,
                cav_pct=cav_pct, ped_vol=ped_vol, w1=round(w1, 2),
                w2=round(1 - w1, 2), vru_w=vru_w, zone=zone,
                min_green=min_green, decision_dt=decision_dt, seed=seed)


def build(fast=False):
    R = []
    # K — VRU multipliers 1.5x, 2x, 3x  (v = m / (1 + m))
    for tv in (2500, 3250):
        for pv in (300, 800):
            for m in (1.5, 2.0, 3.0):
                for sd in SEEDS5:
                    R.append(cfg("K_vrumult", total_veh=tv, ped_vol=pv,
                                 vru_w=round(m / (1 + m), 4), seed=sd))
    # L — penetration sweep 5-20 %, three controllers on identical traffic
    for tv in (1000, 1500, 2500, 3250):
        for cp in (5, 7, 10, 12, 15, 17, 20):
            for sd in SEEDS5:
                R.append(cfg("L_pen_gt", script="v4", mode="gt", total_veh=tv, cav_pct=cp, seed=sd))
                R.append(cfg("L_pen_fixed", strategy="fixedtime", total_veh=tv, cav_pct=cp, seed=sd))
                R.append(cfg("L_pen_cav", script="v4", mode="cav", total_veh=tv, cav_pct=cp, seed=sd))
    for r in R:
        r["ray_dt"] = (5 if fast else 1) if r.get("mode") == "cav" else 0
    return R


def key(r):
    return (f"{r['exp']}__{r['strategy']}_{r['mode'] or 'gt'}_veh{r['total_veh']}_cav{r['cav_pct']}"
            f"_ped{r['ped_vol']}_w1{r['w1']}_vru{r['vru_w']}_detr{r['detect_r']}"
            f"_mg{r['min_green']}_dt{r['decision_dt']}_seed{r['seed']}")


def cmd(r, out):
    c = [PY, "traci_control_v3.py" if r["script"] == "v3" else "traci_control_v4.py",
         "--strategy", r["strategy"], "--w1", str(r["w1"]), "--w2", str(r["w2"]),
         "--total_veh", str(r["total_veh"]), "--cav_pct", str(r["cav_pct"]),
         "--min_green", str(r["min_green"]), "--decision_dt", str(r["decision_dt"]),
         "--vru_w", str(r["vru_w"]), "--ray_dt", str(r["ray_dt"]),
         "--seed", str(r["seed"]), "--out", out]
    if r["ped_vol"] is not None:
        c += ["--ped_vol", str(r["ped_vol"])]
    if r["script"] == "v4":
        c += ["--mode", r["mode"], "--vru_detect_r", str(r["detect_r"])]
    return c


def collect():
    rows = []
    for fn in sorted(os.listdir(RUNS_DIR)):
        if fn.endswith(".csv"):
            try:
                row = list(csv.DictReader(open(os.path.join(RUNS_DIR, fn))))[0]
                row["experiment"] = fn.split("__")[0]
                row["run_id"] = fn[:-4]
                rows.append(row)
            except Exception:
                pass
    if not rows:
        return 0
    front = ["experiment", "mode", "seed", "total_veh", "cav_pct_requested",
             "ped_vol_requested", "w1", "w2", "vru_w", "vru_zone_m", "min_green",
             "decision_dt", "strategy", "vru_detect_r_m", "vru_obs_near_junction_pct", "avg_veh_delay", "avg_ped_delay", "max_veh", "max_ped",
             "jfi", "phase_switches", "insertion_backlog", "teleports",
             "avg_observability_pct", "vru_obs_rays_pct", "vru_obs_rays_plus_zone_pct",
             "zone_vru_detected", "perceived_queue_error", "perceived_wait_error"]
    allk = set().union(*[r.keys() for r in rows])
    cols = [c for c in front if c in allk] + sorted(k for k in allk if k not in front)
    os.makedirs(RES_DIR, exist_ok=True)
    with open(MASTER, "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=cols, extrasaction="ignore")
        w.writeheader()
        for r in rows:
            w.writerow({c: r.get(c, "") for c in cols})
    return len(rows)


def excel():
    try:
        import pandas as pd
        df = pd.read_csv(MASTER)
        with pd.ExcelWriter(MASTER.replace(".csv", ".xlsx"), engine="openpyxl") as xw:
            df.to_excel(xw, sheet_name="ALL", index=False)
            for e, sub in df.groupby("experiment"):
                sub.to_excel(xw, sheet_name=str(e)[:31], index=False)
        print(f"Excel: {MASTER.replace('.csv', '.xlsx')}")
    except Exception as ex:
        print(f"(Excel export skipped: {ex})")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dry_run", action="store_true")
    ap.add_argument("--fast", action="store_true", help="ray_dt 5 for the perception runs")
    ap.add_argument("--jobs", type=int, default=1)
    ap.add_argument("--only", nargs="*", default=None,
                    help="K_vrumult L_pen_gt L_pen_fixed L_pen_cav")
    a = ap.parse_args()
    os.makedirs(RUNS_DIR, exist_ok=True)
    runs = build(a.fast)
    if a.only:
        runs = [r for r in runs if any(r["exp"].startswith(p) for p in a.only)]
    todo = [r for r in runs if not os.path.exists(os.path.join(RUNS_DIR, key(r) + ".csv"))]
    c = Counter(r["exp"] for r in runs)
    print("Configured runs per experiment:")
    for e in sorted(c):
        print(f"   {e:<14} {c[e]}")
    print(f"Total {len(runs)} | already done {len(runs) - len(todo)} | to run {len(todo)}")
    if a.dry_run:
        for r in todo[:3]:
            print("  ", " ".join(cmd(r, os.path.join(RUNS_DIR, key(r) + ".csv"))))
        return

    t0 = time.time(); done = 0
    def job(r):
        out = os.path.join(RUNS_DIR, key(r) + ".csv")
        subprocess.run(cmd(r, out), stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        return r, os.path.exists(out)
    with ThreadPoolExecutor(max_workers=max(1, a.jobs)) as pool:
        for fut in as_completed([pool.submit(job, r) for r in todo]):
            r, ok = fut.result(); done += 1
            el = time.time() - t0
            print(f"[{done}/{len(todo)}] {'ok ' if ok else 'FAILED'} {key(r)} "
                  f"| ETA {el / done * (len(todo) - done) / 60:.0f} min")
            if done % 25 == 0:
                collect()
    n = collect()
    print(f"\nMaster file: {MASTER}  ({n} rows)")
    excel()


if __name__ == "__main__":
    os.environ.setdefault("SUMO_HOME", "/usr/local/lib/python3.12/dist-packages/sumo")
    main()
