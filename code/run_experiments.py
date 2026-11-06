# -*- coding: utf-8 -*-
"""
run_experiments.py  —  one-click batch runner for ALL network-2 experiments.

Same experiment families as network 1, scaled to network 2's higher capacity
and including the corrections learned on network 1:

  A_stress     demand sweep: QAWP vs alternating vs fixedtime (native programme)
  B_mingreen   min-green sweep, levels are multiples of decision_dt (5 s) so
               every level is genuinely distinct
  C_decisiondt decision_dt sweep with min_green = 20 s held so the first
               decision instant is identical -> isolates re-check frequency
  D_rq2        weight sweep x pedestrian volume x demand (RQ1 / RQ2)
  E_penetration  CAV penetration, ground truth vs CAV perception (RQ3)

Every run writes its own row to  runs/  (kept out of your way), and all rows
are collected into  results/master_results_net2.csv  (+ .xlsx with one tab per
experiment).  Re-launching resumes: finished runs are skipped.

>>> SET THE DEMAND LISTS BELOW after running find_capacity.py <<<
The defaults assume a capacity of about 3500 veh/h (measured in testing).

Usage
    python run_experiments.py --dry_run            # list runs, execute nothing
    python run_experiments.py --jobs 4             # run everything
    python run_experiments.py --only A_stress E_penetration --jobs 4
    python run_experiments.py --fast --jobs 4      # ray_dt 5 for the perception runs
"""
import os, sys, csv, time, argparse, subprocess
from concurrent.futures import ThreadPoolExecutor, as_completed

PY = sys.executable
RUNS_DIR = "runs"
RES_DIR = "results"
MASTER = os.path.join(RES_DIR, "master_results_net2.csv")

# ═══════════════════ EDIT THESE AFTER find_capacity.py ═══════════════════════
DEMANDS_STRESS = [1000, 1500, 2000, 2500, 3000, 3250, 3500, 3750, 4000]
DEMAND_LEVELS  = [1500, 2500, 3250]        # low / medium / high  (B, C, E)
DEMANDS_RQ2    = [2500, 3000, 3250]        # congested range where a trade-off exists
PED_VOLS_RQ2   = [120, 300, 800]           # pedestrians/h: quiet / typical / busy
# ═════════════════════════════════════════════════════════════════════════════

SEEDS      = [42, 123, 777]
SEEDS_RQ2  = [42, 123, 777, 2024, 999, 31337, 8675309, 271828, 161803, 112358]
SEEDS_PEN  = SEEDS_RQ2 + [141421, 173205, 223606, 264575, 302300]      # 15 seeds


def cfg(exp, script="v3", strategy="qawp", mode=None, total_veh=None, cav_pct=50,
        ped_vol=None, w1=0.5, min_green=10, decision_dt=5, seed=42):
    return dict(exp=exp, script=script, strategy=strategy, mode=mode,
                total_veh=total_veh, cav_pct=cav_pct, ped_vol=ped_vol,
                w1=round(w1, 2), w2=round(1 - w1, 2),
                min_green=min_green, decision_dt=decision_dt, seed=seed)


def build(fast=False):
    R = []
    for tv in DEMANDS_STRESS:                                   # A
        for sd in SEEDS:
            for st in ("qawp", "alternating", "fixedtime"):
                R.append(cfg("A_stress", strategy=st, total_veh=tv, seed=sd))
    for tv in DEMAND_LEVELS:                                    # B
        for mg in (5, 10, 15, 20, 25, 30):
            for sd in SEEDS:
                R.append(cfg("B_mingreen", total_veh=tv, min_green=mg, seed=sd))
    for tv in DEMAND_LEVELS:                                    # C
        for dt in (1, 2, 4, 5, 10, 20):
            for sd in SEEDS:
                R.append(cfg("C_decisiondt", total_veh=tv, min_green=20, decision_dt=dt, seed=sd))
    for tv in DEMANDS_RQ2:                                      # D
        for pv in PED_VOLS_RQ2:
            for w1 in (0.0, 0.25, 0.5, 0.75, 1.0):
                for sd in SEEDS_RQ2:
                    R.append(cfg("D_rq2", total_veh=tv, ped_vol=pv, w1=w1, seed=sd))
    for tv in DEMAND_LEVELS:                                    # E
        for cp in (20, 40, 60, 80, 100):
            for sd in SEEDS_PEN:
                R.append(cfg("E_penetration_gt", script="v4", mode="gt", total_veh=tv, cav_pct=cp, seed=sd))
                R.append(cfg("E_penetration_cav", script="v4", mode="cav", total_veh=tv, cav_pct=cp, seed=sd))
    for r in R:            # ray casting only where perception or observability matters
        r["ray_dt"] = (5 if fast else 1) if r["exp"].startswith("E_") else 0
    return R


def key(r):
    return (f"{r['exp']}__{r['strategy']}_{r['mode'] or 'gt'}_veh{r['total_veh']}_cav{r['cav_pct']}"
            f"_ped{r['ped_vol']}_w1{r['w1']}_mg{r['min_green']}_dt{r['decision_dt']}_seed{r['seed']}")


def cmd(r, out):
    c = [PY, "traci_control_v3.py" if r["script"] == "v3" else "traci_control_v4.py",
         "--strategy", r["strategy"], "--w1", str(r["w1"]), "--w2", str(r["w2"]),
         "--total_veh", str(r["total_veh"]), "--cav_pct", str(r["cav_pct"]),
         "--min_green", str(r["min_green"]), "--decision_dt", str(r["decision_dt"]),
         "--ray_dt", str(r["ray_dt"]), "--seed", str(r["seed"]), "--out", out]
    if r["ped_vol"] is not None:
        c += ["--ped_vol", str(r["ped_vol"])]
    if r["script"] == "v4":
        c += ["--mode", r["mode"]]
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
    front = ["experiment", "strategy", "mode", "seed", "total_veh", "cav_pct_requested",
             "cav_pct_measured", "ped_vol_requested", "w1", "w2", "min_green", "decision_dt",
             "avg_veh_delay", "avg_ped_delay", "jfi", "los_veh", "los_ped", "phase_switches",
             "insertion_backlog", "teleports", "avg_observability_pct",
             "perceived_queue_error", "perceived_wait_error"]
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
    ap.add_argument("--fast", action="store_true")
    ap.add_argument("--jobs", type=int, default=1)
    ap.add_argument("--only", nargs="*", default=None,
                    help="experiment prefixes to run, e.g. A_stress D_rq2 E_penetration")
    a = ap.parse_args()
    os.makedirs(RUNS_DIR, exist_ok=True)
    runs = build(a.fast)
    if a.only:
        runs = [r for r in runs if any(r["exp"].startswith(p) for p in a.only)]
    todo = [r for r in runs if not os.path.exists(os.path.join(RUNS_DIR, key(r) + ".csv"))]
    from collections import Counter
    c = Counter(r["exp"] for r in runs)
    print("Configured runs per experiment:")
    for e in sorted(c):
        print(f"   {e:<20} {c[e]}")
    print(f"Total {len(runs)} | already done {len(runs) - len(todo)} | to run {len(todo)}")
    if a.dry_run:
        for r in todo[:5]:
            print("  ", " ".join(cmd(r, os.path.join(RUNS_DIR, key(r) + ".csv"))))
        return

    t0 = time.time(); done = 0
    def job(r):
        out = os.path.join(RUNS_DIR, key(r) + ".csv")
        p = subprocess.run(cmd(r, out), stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        return r, os.path.exists(out)
    with ThreadPoolExecutor(max_workers=max(1, a.jobs)) as pool:
        for fut in as_completed([pool.submit(job, r) for r in todo]):
            r, ok = fut.result(); done += 1
            el = time.time() - t0
            print(f"[{done}/{len(todo)}] {'ok ' if ok else 'FAILED'} {key(r)}  "
                  f"| ETA {el / done * (len(todo) - done) / 60:.0f} min")
            if done % 25 == 0:
                collect()
    n = collect()
    print(f"\nMaster file: {MASTER}  ({n} rows)")
    excel()


if __name__ == "__main__":
    os.environ.setdefault("SUMO_HOME", "/usr/local/lib/python3.12/dist-packages/sumo")
    main()
