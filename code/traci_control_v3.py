# -*- coding: utf-8 -*-
"""
traci_control_v3.py  —  QAWP controller using SUMO GROUND TRUTH (network 2 edition).

QAWP reads exact queues and waiting times from SUMO for every road user,
including the route each vehicle intends to take.  This is the perfect-
information upper bound; traci_control_v4.py replaces it with what the CAV
fleet can actually perceive.

Priority of candidate phase p:
    score(p) = w1 * Q(p)/max_Q  +  w2 * T(p)/max_T
The controller holds the current phase for at least min_green seconds, then
every decision_dt seconds switches to the highest-scoring phase — but only if
that phase scores strictly higher than the current one (a tie keeps the
current phase, avoiding pointless switches).

The signal structure (candidate phases, approaches, movement service,
clearance timings) is auto-detected from the network by qawp_common.SignalModel.

Strategies
    qawp         adaptive priority control (this thesis)
    alternating  fixed cycle over the main (through) phases only, fixed green
    fixedtime    the network's own signal programme, run untouched by SUMO

Run examples
    python traci_control_v3.py --strategy qawp --total_veh 3000 --cav_pct 50
    python traci_control_v3.py --strategy fixedtime --total_veh 3000
    python traci_control_v3.py --strategy qawp --min_green 10 --decision_dt 5 --ray_dt 0
"""

import os, sys, argparse

if "SUMO_HOME" in os.environ:
    sys.path.append(os.path.join(os.environ["SUMO_HOME"], "tools"))

import traci
from ray_casting_v2 import FleetPerceptionAggregator
import qawp_common as C


class GroundTruthPerception:
    def __init__(self, model):
        self.m = model
    def phase_state(self):
        return C.gt_phase_state(traci, self.m)
    def update(self, *a, **k):
        pass


class QAWPController:
    def __init__(self, model, actuator, perception, w1, w2, min_green, decision_dt):
        self.m = model; self.act = actuator; self.per = perception
        self.w1 = w1; self.w2 = w2
        self.min_green = min_green; self.decision_dt = decision_dt
        self.switches = 0

    def reset(self):
        self.act.start(self.m.candidates[0])

    def step(self):
        self.act.tick()
        if not self.act.in_green:
            return
        age = self.act.green_age
        if age < self.min_green or age % self.decision_dt != 0:
            return
        q, w = self.per.phase_state()
        mq = max(q.values()) or 1
        mw = max(w.values()) or 1
        score = {p: self.w1 * q[p] / mq + self.w2 * w[p] / mw for p in self.m.candidates}
        cur = self.act.cur
        best = max(score, key=lambda p: (score[p], p == cur))
        if best != cur and score[best] > score[cur] + 1e-9:
            self.act.begin(best)
            self.switches += 1


class AlternatingController:
    """Fixed-time cycle over the main phases only (no protected lefts)."""
    def __init__(self, model, actuator, green=30):
        self.m = model; self.act = actuator; self.green = green
        self.order = sorted(set(model.main_phase.values()))
        self.i = 0; self.switches = 0

    def reset(self):
        self.i = 0
        self.act.start(self.order[0])

    def step(self):
        self.act.tick()
        if self.act.in_green and self.act.green_age >= self.green:
            self.i = (self.i + 1) % len(self.order)
            self.act.begin(self.order[self.i])
            self.switches += 1


class FixedTimeController:
    """The network's own signal programme, left to SUMO untouched."""
    def __init__(self, model):
        self.m = model; self.switches = ""
    def reset(self):
        traci.trafficlight.setProgram(self.m.tls_id, "0")
        traci.trafficlight.setPhase(self.m.tls_id, 0)
    def step(self):
        pass


def build_controller(args, model, perception):
    if args.strategy == "fixedtime":
        return FixedTimeController(model)
    act = C.SignalActuator(traci, model, yellow=args.yellow, ped_clear=args.ped_clear)
    if args.strategy == "alternating":
        return AlternatingController(model, act, green=args.alt_green)
    return QAWPController(model, act, perception, args.w1, args.w2,
                          args.min_green, args.decision_dt)


def run(args, mode_label="gt", perception_factory=None, is_v4=False):
    demand_file, split = C.make_demand_with_split(
        args.demand_file, args.total_veh, args.cav_pct, args.ped_vol)
    model = C.SignalModel(C.net_file_from_cfg("run_simulation.sumocfg"))
    model.vru_w = args.vru_w
    model.vru_zone = args.vru_zone
    model.vru_detect_r = args.vru_detect_r

    traci.start(["sumo-gui" if args.gui else "sumo", "-c", "run_simulation.sumocfg",
                 "--route-files", demand_file, "--seed", str(args.seed),
                 "--time-to-teleport", "100",
                 "--no-warnings", "true", "--no-step-log", "true"])

    title = "CAV PERCEPTION" if is_v4 else "GROUND TRUTH"
    print(f"\n{'=' * 66}\n  {os.path.basename(sys.argv[0])}  [{title}]  mode={mode_label.upper()}")
    print(f"  strategy={args.strategy}  w1={args.w1} w2={args.w2}")
    print(f"  min_green={args.min_green}s  decision_dt={args.decision_dt}s  ray_dt={args.ray_dt}s")
    if split:
        if split[0] is not None:
            print(f"  total={args.total_veh} veh  cav_pct={args.cav_pct}%  (HDV={split[0]} CAV={split[1]})")
    if args.ped_vol is not None:
        print(f"  pedestrians: {args.ped_vol}/h total across 4 crossings")
    if args.vru_w != 0.5:
        print(f"  VRU weight v={args.vru_w}  (one VRU counts {args.vru_w/(1-args.vru_w):.2f} cars)")
    if args.vru_zone > 0:
        print(f"  roadside VRU detection zone: radius {args.vru_zone:.0f} m")
    print(model.describe())
    print(f"{'=' * 66}")

    aggregator = FleetPerceptionAggregator(30.0, 5.0)
    perception = (perception_factory(model) if perception_factory
                  else GroundTruthPerception(model))
    ctrl = build_controller(args, model, perception)
    ctrl.reset()

    veh_delays, ped_delays = [], []
    ever = {"HDV": set(), "pedestrian": set(), "bike_profile": set()}
    seen = {"HDV": set(), "pedestrian": set(), "bike_profile": set()}
    obs_meter = C.ObservabilityMeter()
    teleports = loaded = inserted = 0
    counted, persons = set(), set()
    n_hdv = n_cav = n_bike = 0
    q_err_sum = w_err_sum = 0.0; err_n = 0

    step = 0
    while traci.simulation.getMinExpectedNumber() > 0 and traci.simulation.getTime() < C.SIM_END:
        traci.simulationStep()
        teleports += traci.simulation.getStartingTeleportNumber()
        loaded += traci.simulation.getLoadedNumber()
        inserted += traci.simulation.getDepartedNumber()
        for vid in traci.vehicle.getIDList():
            if vid not in counted:
                counted.add(vid)
                t = traci.vehicle.getTypeID(vid)
                if t.startswith("CAV"): n_cav += 1
                elif t.startswith("HDV"): n_hdv += 1
                elif t.startswith("bike"): n_bike += 1
        for pid in traci.person.getIDList():
            persons.add(pid)

        if args.ray_dt > 0 and step % args.ray_dt == 0:
            obstacles = []
            for vid in traci.vehicle.getIDList():
                x, y = traci.vehicle.getPosition(vid)
                obstacles.append({"id": vid, "type": traci.vehicle.getTypeID(vid), "x": x, "y": y})
            for pid in traci.person.getIDList():
                x, y = traci.person.getPosition(pid)
                obstacles.append({"id": pid, "type": "pedestrian", "x": x, "y": y})
            fleet = aggregator.scan_fleet(obstacles)
            fs = fleet.summary()
            pos = {o["id"]: (o["x"], o["y"]) for o in obstacles}
            for det in fleet.fleet_detections.values():
                det["x"], det["y"] = pos.get(det["object_id"], (0.0, 0.0))
            perception.update(fleet.fleet_detections, step, dt=args.ray_dt)
            for o in obstacles:
                if o["type"] in seen: seen[o["type"]].add(o["id"])
            if fs["n_cavs_active"] > 0:
                for det in fleet.fleet_detections.values():
                    if det["object_type"] in ever:
                        ever[det["object_type"]].add(det["object_id"])
            obs_meter.add(fs)

        ctrl.step()

        if step > 0 and step % 60 == 0:
            for vid in traci.vehicle.getIDList():
                w = traci.vehicle.getAccumulatedWaitingTime(vid)
                if w > 0: veh_delays.append(w)
            for pid in traci.person.getIDList():
                w = traci.person.getWaitingTime(pid)
                if w > 0: ped_delays.append(w)
            if is_v4 and args.mode == "cav":
                pq, pw = perception.phase_state()
                gq, gw = C.gt_phase_state(traci, model)
                n = len(model.candidates)
                q_err_sum += sum(abs(pq[p] - gq[p]) for p in model.candidates) / n
                w_err_sum += sum(abs(pw[p] - gw[p]) for p in model.candidates) / n
                err_n += 1

        if step > 0 and step % 300 == 0:
            av = round(sum(veh_delays) / len(veh_delays), 1) if veh_delays else 0
            ap = round(sum(ped_delays) / len(ped_delays), 1) if ped_delays else 0
            print(f"    t={step:4d}s  veh={av:5.1f}s  ped={ap:5.1f}s  "
                  f"running={traci.simulation.getMinExpectedNumber()}")
        step += 1

    running = traci.simulation.getMinExpectedNumber()
    traci.close()
    if demand_file != args.demand_file:
        try: os.unlink(demand_file)
        except OSError: pass

    all_d = veh_delays + ped_delays
    avg_v = round(sum(veh_delays) / len(veh_delays), 2) if veh_delays else 0
    avg_p = round(sum(ped_delays) / len(ped_delays), 2) if ped_delays else 0
    max_v = round(max(veh_delays, default=0), 1)
    max_p = round(max(ped_delays, default=0), 1)
    backlog = max(0, loaded - inserted)
    total_cars = n_hdv + n_cav
    cav_meas = round(100.0 * n_cav / total_cars, 1) if total_cars else 0.0
    q_err = round(q_err_sum / err_n, 3) if err_n else ""
    w_err = round(w_err_sum / err_n, 3) if err_n else ""

    def fmt(v): return "N/A" if v is None else v
    sim = {"loaded": loaded, "inserted": inserted, "backlog": backlog,
           "backlog_pct": round(100 * backlog / loaded, 1) if loaded else 0,
           "running_at_end": running, "teleports": teleports}
    comp = {"n_hdv": n_hdv, "n_cav": n_cav, "total_cars": total_cars,
            "cav_pct": cav_meas, "n_bike": n_bike, "n_ped": len(persons)}
    ray = {"avg_obs": fmt(obs_meter.average("all")), "obs_hdv": fmt(obs_meter.average("HDV")),
           "obs_ped": fmt(obs_meter.average("ped")), "obs_bike": fmt(obs_meter.average("bike"))}
    perc = {"q_err": q_err, "w_err": w_err} if (is_v4 and args.mode == "cav") else None

    C.print_final_block(f"{args.strategy} | {mode_label.upper()} | w1={args.w1} w2={args.w2}",
                        avg_v, avg_p, C.jfi(all_d), max_v, max_p, C.los(avg_v), C.los(avg_p),
                        sim, ray, perc, comp, getattr(ctrl, "switches", None))

    slug = f"{args.strategy}_{mode_label}_w1{args.w1}_w2{args.w2}"
    if args.total_veh: slug += f"_veh{args.total_veh}_cav{args.cav_pct}"
    if args.ped_vol is not None: slug += f"_ped{args.ped_vol}"
    if args.vru_w != 0.5: slug += f"_vru{args.vru_w}"
    if args.vru_zone > 0: slug += f"_zone{args.vru_zone:g}"
    if args.vru_detect_r > 0: slug += f"_detr{args.vru_detect_r:g}"
    slug += f"_mg{args.min_green}_dt{args.decision_dt}_seed{args.seed}"
    row = {
        "strategy": args.strategy, "mode": mode_label, "seed": args.seed,
        "w1": args.w1, "w2": args.w2,
        "total_veh": args.total_veh, "cav_pct_requested": args.cav_pct,
        "ped_vol_requested": args.ped_vol,
        "min_green": args.min_green, "decision_dt": args.decision_dt,
        "vru_w": args.vru_w, "vru_zone_m": args.vru_zone,
        "vru_detect_r_m": args.vru_detect_r,
        "yellow_s": model.yellow_time if args.yellow is None else args.yellow,
        "ped_clear_s": model.ped_clear_time if args.ped_clear is None else args.ped_clear,
        "phase_switches": getattr(ctrl, "switches", ""),
        "n_hdv": n_hdv, "n_cav": n_cav, "n_bike": n_bike, "n_ped": len(persons),
        "total_cars": total_cars, "cav_pct_measured": cav_meas,
        "avg_veh_delay": avg_v, "avg_ped_delay": avg_p, "jfi": C.jfi(all_d),
        "max_veh": max_v, "max_ped": max_p,
        "los_veh": C.los(avg_v), "los_ped": C.los(avg_p),
        "loaded": loaded, "inserted": inserted, "insertion_backlog": backlog,
        "running_at_end": running, "teleports": teleports,
        **obs_meter.as_dict(),
        "vru_obs_rays_pct": (perception.vru_observability()[0]
                             if hasattr(perception, "vru_observability") else ""),
        "vru_obs_rays_plus_zone_pct": (perception.vru_observability()[1]
                             if hasattr(perception, "vru_observability") else ""),
        "zone_vru_detected": len(getattr(perception, "zone_seen", ())),
        "vru_obs_near_junction_pct": (perception.vru_observability_near()
                             if hasattr(perception, "vru_observability_near") else ""),
        "perceived_queue_error": q_err, "perceived_wait_error": w_err,
        "unique_hdv_detected": len(ever["HDV"]), "unique_hdv_total": len(seen["HDV"]),
        "unique_ped_detected": len(ever["pedestrian"]), "unique_ped_total": len(seen["pedestrian"]),
        "unique_bike_detected": len(ever["bike_profile"]), "unique_bike_total": len(seen["bike_profile"]),
    }
    out = args.out or f"results_{slug}.csv"
    C.write_results_csv(out, row)
    print(f"  Results -> {out}\n")
    return row


def build_parser(is_v4=False):
    p = argparse.ArgumentParser(description="QAWP controller (network-agnostic)")
    p.add_argument("--strategy", choices=["qawp", "alternating", "fixedtime"], default="qawp")
    if is_v4:
        p.add_argument("--mode", choices=["cav", "gt"], default="cav",
                       help="cav = CAV-perceived data drives QAWP; gt = ground truth")
    p.add_argument("--seed", type=int, default=42)
    p.add_argument("--w1", type=float, default=0.5)
    p.add_argument("--w2", type=float, default=0.5)
    p.add_argument("--total_veh", type=int, default=None)
    p.add_argument("--cav_pct", type=float, default=50.0)
    p.add_argument("--ped_vol", type=int, default=None,
                   help="total pedestrians/h across the four crossings")
    p.add_argument("--min_green", type=int, default=10)
    p.add_argument("--decision_dt", type=int, default=5)
    p.add_argument("--ray_dt", type=int, default=1,
                   help="ray-casting interval (s); 0 disables it (ground-truth runs only)")
    p.add_argument("--yellow", type=int, default=None,
                   help="amber seconds (default: read from the network programme)")
    p.add_argument("--ped_clear", type=int, default=None,
                   help="pedestrian clearance seconds (default: read from the programme)")
    p.add_argument("--alt_green", type=int, default=30,
                   help="green seconds per phase for the alternating baseline")
    p.add_argument("--vru_w", type=float, default=0.5,
                   help="relative weight of a VRU (pedestrian or cyclist) against a motor "
                        "vehicle, in (0,1). 0.5 = everyone counts equally (default)")
    p.add_argument("--vru_zone", type=float, default=0.0,
                   help="radius (m) of a roadside sensor detecting all VRUs around the "
                        "junction centre; 0 disables it (CAV perception only, --mode cav)")
    p.add_argument("--vru_detect_r", type=float, default=0.0,
                   help="use CAV detections of VRUs only within this radius (m) of the "
                        "junction centre; 0 = no limit (CAV perception only, --mode cav)")
    p.add_argument("--out", default=None,
                   help="write the result row to this CSV path (used by the batch runner)")
    p.add_argument("--gui", action="store_true")
    return p


if __name__ == "__main__":
    a = build_parser().parse_args()
    os.environ.setdefault("SUMO_HOME", "/usr/local/lib/python3.12/dist-packages/sumo")
    a.demand_file = "demand.rou.xml"
    a.mode = "gt"
    run(a, mode_label="gt")
