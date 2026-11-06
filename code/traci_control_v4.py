# -*- coding: utf-8 -*-
"""
traci_control_v4.py  —  QAWP driven by CAV PERCEPTION (network 2 edition).

Identical to traci_control_v3.py in every respect except where QAWP gets its
queue and waiting-time information.  Here it uses only what the CAV fleet can
realistically know:

  1. CAV SELF-REPORTS (V2X).  Each CAV knows and broadcasts its own position,
     road, speed, accumulated waiting time and intended route.  So a CAV's
     movement is known exactly, even on the 200 m upstream edge.

  2. RAY-CASTING DETECTIONS.  HDVs, cyclists and pedestrians seen by at least
     one CAV sensor (72 rays, 30 m, occlusion) have a known POSITION only.
       * waiting time is estimated by tracking the object across scans: it
         accumulates while the measured speed is <= 0.1 m/s and resets as soon
         as the object moves (SUMO's own definition), with a small braking
         correction when the stop itself was observed;
       * position is map-matched to a lane (CAVs carry an HD map);
       * the intended movement is NOT observable.  A road user in a turn-pocket
         lane is credited to the phases guaranteed to release that lane (e.g.
         the left-only pocket lane -> protected-left phase).  A road user still
         on the upstream edge, before the lanes separate, is credited to its
         approach's main (through) phase, since its intent cannot be known;
       * a waiting pedestrian is credited to the phase releasing the nearest
         crossing.

  Road users no CAV can see do not exist for the controller.  That, plus the
  intent uncertainty on upstream edges, is the observability effect RQ3 studies.

  --mode cav  perception drives QAWP (default, the realistic controller)
  --mode gt   ground truth drives QAWP (identical to v3; for A/B comparison)

Run examples
    python traci_control_v4.py --total_veh 3000 --cav_pct 30 --mode cav
    python traci_control_v4.py --total_veh 3000 --cav_pct 30 --mode gt
"""

import os, sys, math

if "SUMO_HOME" in os.environ:
    sys.path.append(os.path.join(os.environ["SUMO_HOME"], "tools"))

import traci
import qawp_common as C
import traci_control_v3 as V3

# ── perception tuning ────────────────────────────────────────────────────────
STATIONARY_WINDOW  = 3      # consecutive waiting scans before counted as queued
OBJECT_STALE_AGE   = 30     # s since last seen -> drop from memory
A_BRAKE            = 4.5    # m/s^2 nominal braking (HDV decel in the demand file)
GAP_MOVE_TOL       = 0.3    # m: displacement after an occlusion gap that counts as movement
NEAR_R             = 50     # m: radius for the near-junction VRU observability measure
WAIT_MEMORY        = 100    # s: memory window for accumulated waiting time (SUMO default)
LANE_MATCH_RADIUS  = 3.5    # m: max distance from a lane centre-line to match it
PED_MATCH_RADIUS   = 6.0    # m: max distance to a sidewalk / walking-area outline


class _Tracked:
    """One road user as perceived across successive scans."""
    __slots__ = ("x", "y", "otype", "last_seen", "speed", "seen_moving",
                 "wait_steps", "est_wait", "self_rep", "stops")

    def __init__(self, x, y, otype, t):
        self.x = x; self.y = y; self.otype = otype; self.last_seen = t
        self.speed = None; self.seen_moving = False
        self.wait_steps = 0; self.est_wait = 0.0
        self.self_rep = False      # True for CAVs: speed and wait are broadcast, not inferred
        self.stops = []            # (time, seconds stopped) inside the memory window

    def update(self, x, y, t, dt):
        dt_obs = max(1, t - self.last_seen) if self.last_seen is not None else dt
        moved = math.hypot(x - self.x, y - self.y)
        v = moved / dt_obs
        self.x = x; self.y = y; self.last_seen = t
        # Continuous observation: judge by speed (SUMO's 0.1 m/s rule).
        # After an occlusion gap, average speed hides short movements (a queue
        # creeping 1.5 m in 20 s averages 0.075 m/s), so ANY displacement above
        # GAP_MOVE_TOL means the road user moved and its waiting clock restarts.
        moved_flag = (v > C.WAIT_SPEED) if dt_obs <= dt else (moved > GAP_MOVE_TOL)
        if moved_flag:
            self.speed = max(v, C.WAIT_SPEED + 0.01) if dt_obs > dt else v
            self.seen_moving = True
            self.wait_steps = 0; self.est_wait = 0.0
        else:
            if self.wait_steps == 0 and self.seen_moving and self.speed:
                self.stops.append((t, self.speed / A_BRAKE))   # braking already elapsed
            self.wait_steps += 1
            self.stops.append((t, dt_obs))
            self.speed = 0.0
        # Waiting time is accumulated over a rolling memory window and is NOT
        # cleared when the road user creeps forward, matching the definition
        # SUMO uses for accumulated waiting time (and hence the ground truth).
        self.stops = [(ts, v) for ts, v in self.stops if t - ts <= WAIT_MEMORY]
        self.est_wait = sum(v for _, v in self.stops)

    @property
    def is_queued(self):
        # A connected vehicle broadcasts its own speed, so one observation is
        # enough. For a road user whose stillness is INFERRED from successive
        # ray detections, several consecutive observations are required so that
        # a momentary pause is not read as a queue.
        return self.wait_steps >= (1 if self.self_rep else STATIONARY_WINDOW)


class CAVPerceptionLayer:
    def __init__(self, model):
        self.m = model
        self._buf = {}
        self._cav_road = {}
        self._cav_route = {}
        # every edge of every approach chain -> its junction-incoming edge
        self.edge_to_approach = {e: a for a, ch in model.chains.items() for e in ch}
        # lane centre-lines of the approach chains, for map matching
        self._lanes = []
        for a, ch in model.chains.items():
            for e in ch:
                for ln in model.net.getEdge(e).getLanes():
                    self._lanes.append((ln.getID(), e, ln.getShape()))
        # pedestrian surfaces (sidewalks + kerb walking areas) for map matching
        self._ped_lanes = model.ped_lanes
        # optional roadside VRU detection zone (infrastructure sensor)
        self.zone_r = float(getattr(model, "vru_zone", 0.0) or 0.0)
        self.centre = model.net.getNode(model.tls_id).getCoord()
        self.zone_seen = set()              # VRUs ever detected by the zone sensor
        # optional limit on how far from the junction CAV detections of VRUs are
        # used at all (0 = no limit): VRUs seen further away are ignored
        self.detect_r = float(getattr(model, "vru_detect_r", 0.0) or 0.0)
        self.near_r = self.detect_r if self.detect_r > 0 else NEAR_R
        self._obs_near = 0.0; self._near_n = 0
        self._obs_ray = self._obs_comb = 0.0
        self._obs_n = 0

    # ── called every ray-casting scan ────────────────────────────────────────
    def update(self, fleet_detections, t, dt=1):
        # 1. CAV self-reports (exact, via V2X)
        for vid in traci.vehicle.getIDList():
            if not traci.vehicle.getTypeID(vid).startswith("CAV"):
                continue
            x, y = traci.vehicle.getPosition(vid)
            rec = self._buf.get(vid) or _Tracked(x, y, "CAV", t)
            rec.x = x; rec.y = y; rec.last_seen = t
            rec.speed = traci.vehicle.getSpeed(vid)
            rec.est_wait = traci.vehicle.getAccumulatedWaitingTime(vid)
            rec.self_rep = True
            rec.wait_steps = rec.wait_steps + 1 if rec.speed <= C.WAIT_SPEED else 0
            self._buf[vid] = rec
            self._cav_road[vid] = traci.vehicle.getRoadID(vid)
            self._cav_route[vid] = traci.vehicle.getRoute(vid)
        # 2. ray-casting detections (position only)
        updated = set()
        cx, cy = self.centre
        for det in fleet_detections.values():
            if det["object_type"] == "CAV":
                continue
            if (self.detect_r > 0 and det["object_type"] in ("pedestrian", "bike_profile")
                    and math.hypot(det["x"] - cx, det["y"] - cy) > self.detect_r):
                continue                     # VRU outside the decision-relevant area
            oid = det["object_id"]
            rec = self._buf.get(oid)
            if rec is None:
                self._buf[oid] = _Tracked(det["x"], det["y"], det["object_type"], t)
            else:
                rec.update(det["x"], det["y"], t, dt)
            updated.add(oid)

        # 2b. optional roadside VRU zone: a fixed sensor at the junction detects
        #     every pedestrian and cyclist within zone_r metres of its centre,
        #     independently of the CAV fleet.  Position only, exactly like a ray
        #     detection, so the same tracking and waiting estimation applies.
        #     An object already updated from rays this scan is not updated twice.
        vrus = {}
        for pid in traci.person.getIDList():
            vrus[pid] = (traci.person.getPosition(pid), "pedestrian")
        for vid in traci.vehicle.getIDList():
            if traci.vehicle.getTypeID(vid).startswith(C.VRU_VEHICLE_PREFIX):
                vrus[vid] = (traci.vehicle.getPosition(vid), "bike_profile")
        in_zone = set()
        if self.zone_r > 0:
            for oid, ((x, y), typ) in vrus.items():
                if math.hypot(x - cx, y - cy) <= self.zone_r:
                    in_zone.add(oid)
                    self.zone_seen.add(oid)
                    if oid in updated:
                        continue
                    rec = self._buf.get(oid)
                    if rec is None:
                        self._buf[oid] = _Tracked(x, y, typ, t)
                    else:
                        rec.update(x, y, t, dt)
        # VRU observability: rays alone, and rays + zone (share of all VRUs)
        # observability of VRUs NEAR the junction (the ones that matter for control)
        near = {o for o, ((x, y), _) in vrus.items() if math.hypot(x - cx, y - cy) <= self.near_r}
        if near:
            self._obs_near += 100.0 * len(near & updated) / len(near)
            self._near_n += 1
        if vrus:
            ray_vru = {o for o in updated if o in vrus}
            self._obs_ray  += 100.0 * len(ray_vru) / len(vrus)
            self._obs_comb += 100.0 * len(ray_vru | in_zone) / len(vrus)
            self._obs_n += 1

        # 3. forget road users not seen recently
        for oid in [k for k, r in self._buf.items() if t - r.last_seen > OBJECT_STALE_AGE]:
            self._buf.pop(oid, None)
            self._cav_road.pop(oid, None); self._cav_route.pop(oid, None)

    # ── map matching ─────────────────────────────────────────────────────────
    @staticmethod
    def _seg_dist(px, py, shape):
        best = math.inf
        for (ax, ay), (bx, by) in zip(shape, shape[1:]):
            dx, dy = bx - ax, by - ay
            L2 = dx * dx + dy * dy
            t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / L2))
            d = math.hypot(px - (ax + t * dx), py - (ay + t * dy))
            if d < best:
                best = d
        return best

    def _match_lane(self, x, y):
        best, best_d = None, LANE_MATCH_RADIUS
        for lid, e, sh in self._lanes:
            d = self._seg_dist(x, y, sh)
            if d < best_d:
                best_d, best = d, (lid, e)
        return best

    def _pedestrian_phases(self, x, y):
        """
        Same rule as ground truth: find the sidewalk or walking area the
        pedestrian stands on (map matching), then the crossing it leads to.
        """
        best, best_d = None, PED_MATCH_RADIUS
        for eid, sh in self._ped_lanes:
            d = self._seg_dist(x, y, sh)
            if d < best_d:
                best_d, best = d, eid
        return self.m.ped_edge_phases.get(best, set()) if best else set()

    # ── which phases would release this perceived road user ─────────────────
    def _phases_for(self, oid, rec):
        if rec.otype == "CAV":
            a = self.edge_to_approach.get(self._cav_road.get(oid))
            if a is None:
                return set()
            route = self._cav_route.get(oid, ())
            out = route[route.index(a) + 1] if a in route and route.index(a) + 1 < len(route) else None
            return self.m.served_by.get((a, out)) or {self.m.main_phase[a]}
        if rec.otype == "pedestrian":
            return self._pedestrian_phases(rec.x, rec.y)
        hit = self._match_lane(rec.x, rec.y)
        if hit is None:
            return set()
        lane, edge = hit
        a = self.edge_to_approach.get(edge)
        if a is None:
            return set()
        if edge == a:                                   # in the turn pockets
            return self.m.lane_phases.get(lane) or {self.m.main_phase[a]}
        return {self.m.main_phase[a]}                   # upstream: intent unknown

    def phase_state(self):
        q = {p: 0.0 for p in self.m.candidates}
        w = {p: 0.0 for p in self.m.candidates}
        for oid, rec in self._buf.items():
            if not rec.is_queued:
                continue
            cw = C.class_weight(self.m, rec.otype in ("pedestrian", "bike_profile"))
            for p in self._phases_for(oid, rec):
                q[p] += cw
                if cw * rec.est_wait > w[p]:
                    w[p] = cw * rec.est_wait
        return q, w

    def vru_observability(self):
        n = self._obs_n
        return (round(self._obs_ray / n, 1) if n else "",
                round(self._obs_comb / n, 1) if n else "")

    def vru_observability_near(self):
        return round(self._obs_near / self._near_n, 1) if self._near_n else ""


if __name__ == "__main__":
    a = V3.build_parser(is_v4=True).parse_args()
    os.environ.setdefault("SUMO_HOME", "/usr/local/lib/python3.12/dist-packages/sumo")
    a.demand_file = "demand.rou.xml"
    if a.mode == "cav" and a.ray_dt <= 0:
        sys.exit("ERROR: --mode cav needs ray casting; use --ray_dt 1 (or 5).")
    if a.mode == "cav":
        V3.run(a, mode_label="cav", perception_factory=CAVPerceptionLayer, is_v4=True)
    else:
        V3.run(a, mode_label="gt", is_v4=True)
