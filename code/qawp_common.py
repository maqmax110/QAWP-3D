# -*- coding: utf-8 -*-
"""
qawp_common.py  —  shared core for the QAWP controllers (network 2 edition).

This version is NETWORK-AGNOSTIC.  Instead of hard-coding edge names and phase
indices, it reads the signal program and lane connections from the network
file and derives everything the controller needs:

    * which phases are genuine "green" phases a controller may choose
    * which approach edges feed the junction, including split approaches
      (e.g. a 200 m upstream edge followed by a turn-pocket edge)
    * which phase(s) serve each traffic movement
    * which phase releases each pedestrian crossing

It therefore runs unchanged on network 1 (2 green phases) and network 2
(4 green phases incl. protected left turns).

MEASUREMENT CODE (delay, JFI, LoS, backlog, teleports, observability) is kept
identical in definition to the network-1 edition so results stay comparable.

------------------------------------------------------------------------------
KEY DESIGN DECISIONS (state these in the thesis methodology)
------------------------------------------------------------------------------
1. Candidate phases.  A phase is a candidate if it contains no yellow and is
   not dominated by another green phase.  A phase is dominated when another
   green phase gives protected ('G') green to a superset of its links and
   green of any kind to a superset of its links.  This removes pure
   pedestrian-clearance variants (e.g. ph1 = ph0 with crossings red) while
   keeping protected-left phases (their 'G' links are only 'g' elsewhere).

2. "Best service" crediting.  A movement is credited only to the phase(s)
   giving it its highest service level ('G' protected > 'g' permissive).
   Without this, a permissive-left main phase would always out-score the
   protected-left phase and the protected phase would never be selected.

3. Queue Q(phase)  = number of WAITING road users (speed <= 0.1 m/s) that the
   phase would release: cars, cyclists and pedestrians, each counted as one.
   Waiting-time T(phase) = the LONGEST such wait.  Queue is summed over all
   movements a phase serves; waiting time is the maximum (Max-Pressure-style).

4. Clearance.  Leaving a green phase uses the SAME intervals the network's
   own signal program uses: a pedestrian-clearance interval (crossings red,
   vehicles still green) when a crossing loses its green, then an amber
   interval for vehicle links that lose their green.  Interval lengths are
   read from the network program, so clearance is realistic, safe for any
   phase-to-phase transition, and identical for every strategy compared.
"""

import os, re, csv, tempfile, math

SIM_END    = 3600        # hard stop (s) — keeps every run comparable
WAIT_SPEED = 0.1         # m/s — at or below this a road user is "waiting" (SUMO convention)


# ── Level of Service (HCM/HBS control-delay thresholds) ─────────────────────
def los(delay):
    for grade, lo, hi in [("A", 0, 10), ("B", 10, 20), ("C", 20, 35),
                          ("D", 35, 55), ("E", 55, 80), ("F", 80, 1e9)]:
        if lo <= delay < hi:
            return grade
    return "F"


# ── Jain's Fairness Index ────────────────────────────────────────────────────
def jfi(values):
    v = [x for x in values if x > 0]
    if not v:
        return None
    s = sum(v); s2 = sum(x * x for x in v)
    return round(s * s / (len(v) * s2), 4) if s2 else 1.0


def net_file_from_cfg(cfg="run_simulation.sumocfg"):
    txt = open(cfg, encoding="utf-8").read()
    m = re.search(r'<net-file\s+value="([^"]+)"', txt)
    if not m:
        raise RuntimeError(f"no <net-file> in {cfg}")
    return os.path.join(os.path.dirname(os.path.abspath(cfg)), m.group(1))


# ═════════════════════════════════════════════════════════════════════════════
# SignalModel — everything derived from the network file
# ═════════════════════════════════════════════════════════════════════════════
class SignalModel:
    def __init__(self, net_file, tls_id=None):
        import sumolib
        self.net = sumolib.net.readNet(net_file, withInternal=True, withPrograms=True,
                                       withPedestrianConnections=True)
        tlss = self.net.getTrafficLights()
        if not tlss:
            raise RuntimeError("network has no traffic light")
        tls = (next(t for t in tlss if t.getID() == tls_id)
               if tls_id else tlss[0])
        self.tls_id = tls.getID()

        # ── program phases (first program) ─────────────────────────────────
        prog = list(tls.getPrograms().values())[0]
        self.phases    = [p.state for p in prog.getPhases()]
        self.durations = [float(p.duration) for p in prog.getPhases()]
        self.n_links   = len(self.phases[0])

        # ── controlled links ────────────────────────────────────────────────
        self.link_in  = {}      # link index -> incoming edge id
        self.link_out = {}      # link index -> outgoing edge id
        self.crossing_links = {}  # crossing edge id -> link index
        for inL, outL, li in tls.getConnections():
            ie = inL.getEdge().getID(); oe = outL.getEdge().getID()
            if oe.startswith(":") and "_c" in oe:
                self.crossing_links[oe] = li
            elif not ie.startswith(":"):
                self.link_in[li] = ie
                self.link_out[li] = oe
        self.vehicle_links = sorted(self.link_in)

        # ── candidate green phases ──────────────────────────────────────────
        self.candidates = self._detect_candidates()

        # ── clearance interval lengths, read from the program ──────────────
        self.yellow_time    = self._detect_yellow_time()
        self.ped_clear_time = self._detect_ped_clear_time()

        # ── approach chains (upstream edge(s) + junction-incoming edge) ────
        self.approaches = sorted(set(self.link_in.values()))
        self.chains = {a: self._chain(a) for a in self.approaches}

        # ── movements and which phases serve them ───────────────────────────
        self.served_by = self._movement_service()
        self.main_phase = self._main_phases()
        self.crossing_phases = {
            c: self._best_phases([li]) for c, li in self.crossing_links.items()}

        # ── lane -> phases, for sensor-based (perceived) attribution ───────
        self.lane_phases = self._lane_phases()

        # ── pedestrian edge -> phases releasing the crossing it leads to ───
        self.ped_edge_phases, self.ped_lanes = self._pedestrian_map()

    # ── helpers ──────────────────────────────────────────────────────────────
    @staticmethod
    def _lvl(ch):
        return 2 if ch == "G" else 1 if ch == "g" else 0

    def _green_sets(self, i):
        s = self.phases[i]
        return ({k for k, c in enumerate(s) if c == "G"},
                {k for k, c in enumerate(s) if c in "Gg"})

    def _detect_candidates(self):
        green = [i for i, s in enumerate(self.phases)
                 if "y" not in s and "Y" not in s
                 and any(self._lvl(s[li]) for li in self.vehicle_links)]
        keep = []
        for i in green:
            Gi, Ai = self._green_sets(i)
            dominated = False
            for j in green:
                if j == i:
                    continue
                Gj, Aj = self._green_sets(j)
                if Gi <= Gj and Ai <= Aj and (Gi, Ai) != (Gj, Aj):
                    dominated = True; break
                # identical twin: keep the earlier one only
                if (Gi, Ai) == (Gj, Aj) and j < i:
                    dominated = True; break
            if not dominated:
                keep.append(i)
        return keep

    def _detect_yellow_time(self):
        ys = [d for s, d in zip(self.phases, self.durations) if "y" in s]
        return int(round(min(ys))) if ys else 3

    def _detect_ped_clear_time(self):
        """Duration of a phase that equals a green phase with crossings red."""
        cross = set(self.crossing_links.values())
        best = 0
        for i, s in enumerate(self.phases):
            for j, t in enumerate(self.phases):
                if i == j or "y" in s or "y" in t:
                    continue
                diff = [k for k in range(self.n_links) if s[k] != t[k]]
                if diff and all(k in cross for k in diff) \
                   and all(s[k] in "Gg" and t[k] == "r" for k in diff):
                    best = max(best, int(round(self.durations[j])))
        return best

    def _chain(self, junction_edge):
        """[upstream..., junction_edge] following single-predecessor splits."""
        chain = [junction_edge]
        e = self.net.getEdge(junction_edge)
        seen = {junction_edge}
        while True:
            node = e.getFromNode()
            ins = [x for x in node.getIncoming() if not x.getID().startswith(":")]
            outs = [x for x in node.getOutgoing() if not x.getID().startswith(":")]
            # a plain split node: one road in, one road out, no signal, and not
            # a network boundary (a dead_end node's "incoming" edge is the
            # opposite-direction road leaving the network, not an upstream).
            if (len(ins) == 1 and len(outs) == 1
                    and node.getType() not in ("traffic_light", "dead_end")):
                prev = ins[0]
                if prev.getID() in seen:
                    break
                chain.insert(0, prev.getID()); seen.add(prev.getID())
                e = prev
            else:
                break
        return chain

    def _best_phases(self, links):
        lv = {p: max(self._lvl(self.phases[p][li]) for li in links)
              for p in self.candidates}
        top = max(lv.values()) if lv else 0
        return {p for p, v in lv.items() if v == top and v > 0}

    def _movement_service(self):
        mv = {}
        for li in self.vehicle_links:
            mv.setdefault((self.link_in[li], self.link_out[li]), []).append(li)
        return {m: self._best_phases(links) for m, links in mv.items()}

    def _main_phases(self):
        """Phase serving the most movements of each approach (its 'through' phase)."""
        out = {}
        for a in self.approaches:
            cnt = {}
            for (ie, oe), ps in self.served_by.items():
                if ie == a:
                    for p in ps:
                        cnt[p] = cnt.get(p, 0) + 1
            out[a] = max(cnt, key=lambda p: (cnt[p], -p)) if cnt else self.candidates[0]
        return out

    def _lane_phases(self):
        """
        For each lane of each junction-incoming edge: phases GUARANTEED to
        release a road user standing in that lane, i.e. the intersection of
        the service sets of every movement reachable from that lane.  Used when
        a sensor sees a road user's position but not its intended route.
        """
        lp = {}
        by_lane = {}
        tls = [t for t in self.net.getTrafficLights() if t.getID() == self.tls_id][0]
        for inL, outL, li in tls.getConnections():
            if li not in self.link_in:
                continue
            by_lane.setdefault(inL.getID(), set()).add(
                (self.link_in[li], self.link_out[li]))
        for lane, moves in by_lane.items():
            sets = [self.served_by[m] for m in moves if m in self.served_by]
            inter = set.intersection(*sets) if sets else set()
            lp[lane] = inter
        return lp

    def _pedestrian_map(self):
        """
        Which crossing will a pedestrian standing on a given sidewalk or walking
        area use next?  Found by searching the pedestrian network forward to the
        first signal-controlled crossing.  At this junction every walking area
        feeds exactly one crossing, so the answer depends only on WHERE the
        pedestrian is standing, not on its destination.  Ground truth and CAV
        perception therefore apply the identical rule, and differ only in which
        pedestrians the fleet can see.
        """
        import collections
        node = self.tls_id
        g = collections.defaultdict(set)
        for e in self.net.getEdges(withInternal=True):
            for ln in e.getLanes():
                if not ln.allows("pedestrian"):
                    continue
                for c in ln.getOutgoing():
                    tl = c.getToLane()
                    if tl.allows("pedestrian"):
                        g[e.getID()].add(tl.getEdge().getID())

        def first_crossing(start):
            seen = {start}; q = collections.deque([start])
            while q:
                u = q.popleft()
                for v in sorted(g[u]):
                    if v in self.crossing_links:
                        return v
                    if v not in seen:
                        seen.add(v); q.append(v)
            return None

        edges = {e for ch in self.chains.values() for e in ch}
        edges |= {e.getID() for e in self.net.getEdges(withInternal=True)
                  if e.getID().startswith(f":{node}_w")}
        phases, lanes = {}, []
        for eid in edges:
            c = first_crossing(eid)
            if c is None or c not in self.crossing_phases:
                continue
            phases[eid] = self.crossing_phases[c]
            for ln in self.net.getEdge(eid).getLanes():
                if ln.allows("pedestrian"):
                    lanes.append((eid, ln.getShape()))
        return phases, lanes

    # ── describe (printed once per run for traceability) ───────────────────
    def describe(self):
        lines = [f"  signal {self.tls_id}: {len(self.phases)} programme phases, "
                 f"candidate green phases = {self.candidates}",
                 f"  clearance: pedestrian {self.ped_clear_time}s + amber {self.yellow_time}s"]
        for a in self.approaches:
            lines.append(f"  approach {'->'.join(self.chains[a])}  main phase {self.main_phase[a]}")
        return "\n".join(lines)


# ═════════════════════════════════════════════════════════════════════════════
# SignalActuator — safe, identical transitions for every strategy
# ═════════════════════════════════════════════════════════════════════════════
class SignalActuator:
    """
    Drives the signal with setRedYellowGreenState so any candidate phase can
    follow any other.  A transition from phase A to phase B is:

      1. PEDESTRIAN CLEARANCE  (only if a crossing loses its green)
         crossings that are green in A but not in B go red; everything else
         keeps A's state.  Lasts ped_clear seconds.
      2. AMBER  (only if a vehicle link loses its green)
         vehicle links green in A but not in B show amber; links green in both
         stay green; everything else red.  Lasts yellow seconds.
      3. GREEN of B.
    """
    def __init__(self, traci, model, yellow=None, ped_clear=None):
        self.traci = traci
        self.m = model
        self.yellow = model.yellow_time if yellow is None else yellow
        self.ped_clear = model.ped_clear_time if ped_clear is None else ped_clear
        self.cur = None
        self.target = None
        self.stage = "green"
        self.t = 0
        self.green_age = 0

    def _set(self, state):
        self.traci.trafficlight.setRedYellowGreenState(self.m.tls_id, state)

    def start(self, phase):
        self.cur = phase; self.target = None
        self.stage = "green"; self.t = 0; self.green_age = 0
        self._set(self.m.phases[phase])

    @property
    def in_green(self):
        return self.stage == "green"

    def begin(self, target):
        """Start a transition from the current green phase to `target`."""
        A = self.m.phases[self.cur]; B = self.m.phases[target]
        self.target = target
        cross = set(self.m.crossing_links.values())
        lose_x = [k for k in cross if A[k] in "Gg" and B[k] not in "Gg"]
        if lose_x and self.ped_clear > 0:
            s = list(A)
            for k in lose_x:
                s[k] = "r"
            self.stage = "pedclear"; self.t = 0; self._set("".join(s))
            return
        self._to_amber()

    def _to_amber(self):
        A = self.m.phases[self.cur]; B = self.m.phases[self.target]
        cross = set(self.m.crossing_links.values())
        s = []
        lost = False
        for k in range(len(A)):
            a, b = A[k], B[k]
            if a in "Gg" and b in "Gg":
                s.append(a)                 # continuing movement keeps green
            elif a in "Gg" and k in cross:
                s.append("r")               # pedestrian signals have no amber:
                                            # a crossing losing green goes red
                                            # (already red if pedclear ran)
            elif a in "Gg":
                s.append("y"); lost = True  # vehicle link losing green -> amber
            else:
                s.append("r")
        if lost and self.yellow > 0:
            self.stage = "amber"; self.t = 0; self._set("".join(s))
        else:
            self._to_green()

    def _to_green(self):
        self.cur = self.target; self.target = None
        self.stage = "green"; self.t = 0; self.green_age = 0
        self._set(self.m.phases[self.cur])

    def tick(self):
        """Advance one simulation second."""
        self.t += 1
        if self.stage == "green":
            self.green_age += 1
        elif self.stage == "pedclear" and self.t >= self.ped_clear:
            self._to_amber()
        elif self.stage == "amber" and self.t >= self.yellow:
            self._to_green()


# ═════════════════════════════════════════════════════════════════════════════
# Ground-truth traffic state (SUMO omniscient view)
# ═════════════════════════════════════════════════════════════════════════════
def _exit_edge(traci, vid, approach):
    try:
        route = traci.vehicle.getRoute(vid)
        i = route.index(approach)
        return route[i + 1] if i + 1 < len(route) else None
    except Exception:
        return None



# ── Road-user class weighting (vehicles vs vulnerable road users) ───────────
VRU_VEHICLE_PREFIX = "bike"          # cyclists are SUMO vehicles of type bike_profile

def class_weight(model, is_vru):
    """
    Relative weight of one road user in BOTH the queue and the waiting-time
    term.  model.vru_w = v in (0, 1):  a VRU counts v, a motor vehicle 1 - v.
    v = 0.5 weights everyone equally and reproduces the unweighted controller
    exactly, because the score normalises each term by its maximum.
    v = 0.8 makes one pedestrian or cyclist count four times one car.
    """
    v = getattr(model, "vru_w", 0.5)
    return v if is_vru else 1.0 - v

def gt_phase_state(traci, model):
    """
    Exact Q and T per candidate phase from SUMO ground truth.  Vehicles and
    cyclists anywhere on an approach chain count, with their movement taken
    from their route; pedestrians are credited to the phase releasing the
    crossing they are waiting for.
    """
    q = {p: 0.0 for p in model.candidates}
    w = {p: 0.0 for p in model.candidates}
    for a, chain in model.chains.items():
        for e in chain:
            try:
                vids = traci.edge.getLastStepVehicleIDs(e)
            except Exception:
                continue
            for vid in vids:
                if traci.vehicle.getSpeed(vid) > WAIT_SPEED:
                    continue
                ps = model.served_by.get((a, _exit_edge(traci, vid, a)))
                if not ps:
                    ps = {model.main_phase[a]}
                cw = class_weight(model, traci.vehicle.getTypeID(vid).startswith(VRU_VEHICLE_PREFIX))
                wt = cw * traci.vehicle.getAccumulatedWaitingTime(vid)
                for p in ps:
                    q[p] += cw
                    if wt > w[p]:
                        w[p] = wt
    for pid in traci.person.getIDList():
        try:
            if traci.person.getSpeed(pid) > WAIT_SPEED:
                continue
            # the crossing this pedestrian is queued for follows from where it
            # stands: kerb walking area OR the approach sidewalk behind it
            ps = model.ped_edge_phases.get(traci.person.getRoadID(pid))
            if not ps:
                continue
            cw = class_weight(model, True)
            wt = cw * traci.person.getWaitingTime(pid)
        except Exception:
            continue
        for p in ps:
            q[p] += cw
            if wt > w[p]:
                w[p] = wt
    return q, w


def gt_queue_by_approach(traci, model):
    """Total waiting road users per approach (used only for perception error)."""
    out = {}
    for a, chain in model.chains.items():
        n = 0
        for e in chain:
            try:
                n += sum(1 for v in traci.edge.getLastStepVehicleIDs(e)
                         if traci.vehicle.getSpeed(v) <= WAIT_SPEED)
            except Exception:
                pass
        out[a] = n
    return out


# ── Per-type observability accumulator (unchanged from network 1) ───────────
class ObservabilityMeter:
    KEYS = [("all", "observability_pct"), ("HDV", "observability_pct_HDV"),
            ("ped", "observability_pct_ped"), ("bike", "observability_pct_bike")]

    def __init__(self):
        self._sum = {k: 0.0 for k, _ in self.KEYS}
        self._n = {k: 0 for k, _ in self.KEYS}

    def add(self, fs):
        for short, field in self.KEYS:
            v = fs.get(field)
            if v is not None:
                self._sum[short] += v; self._n[short] += 1

    def average(self, short):
        n = self._n[short]
        return round(self._sum[short] / n, 1) if n else None

    def as_dict(self):
        name = {"all": "avg_observability_pct", "HDV": "avg_observability_pct_HDV",
                "ped": "avg_observability_pct_ped", "bike": "avg_observability_pct_bike"}
        out = {}
        for short, _ in self.KEYS:
            v = self.average(short)
            out[name[short]] = "" if v is None else v
        return out


# ── Demand-file rewriter (--total_veh / --cav_pct / --ped_vol) ──────────────
def make_demand_with_split(base, total_veh, cav_pct, ped_vol=None):
    if total_veh is None and ped_vol is None:
        return base, None
    text = open(base, encoding="utf-8").read()
    n_hdv = n_cav = None
    if total_veh is not None:
        n_cav = round(total_veh * cav_pct / 100.0)
        n_hdv = total_veh - n_cav
        text = re.sub(r'(id="hdv_random_flow"[^>]*?number=")\d+(")',
                      rf'\g<1>{n_hdv}\g<2>', text)
        text = re.sub(r'(id="cav_random_flow"[^>]*?number=")\d+(")',
                      rf'\g<1>{n_cav}\g<2>', text)
    if ped_vol is not None:
        per = max(1, round(ped_vol / 4.0))
        text = re.sub(r'(<personFlow[^>]*?number=")\d+(")', rf'\g<1>{per}\g<2>', text)
    tmp = tempfile.NamedTemporaryFile(mode="w", suffix=".rou.xml",
                                      delete=False, encoding="utf-8",
                                      dir=os.getcwd())
    tmp.write(text); tmp.close()
    return tmp.name, (n_hdv, n_cav)


def write_results_csv(path, row):
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(row.keys()))
        w.writeheader(); w.writerow(row)


def print_final_block(title, avg_v, avg_p, jfi_val, max_v, max_p, los_v, los_p,
                      sim, ray=None, perc=None, comp=None, switches=None):
    print(f"\n{'=' * 66}\n  FINAL RESULTS — {title}\n{'=' * 66}")
    print(f"  Avg vehicle delay      : {avg_v} s     (LoS {los_v})")
    print(f"  Avg pedestrian delay   : {avg_p} s     (LoS {los_p})")
    print(f"  Jain's Fairness Index  : {jfi_val}")
    print(f"  Max vehicle delay      : {max_v} s")
    print(f"  Max pedestrian delay   : {max_p} s")
    if switches is not None:
        print(f"  Phase switches         : {switches}")
    print(f"\n  Demand completion")
    print(f"    Scheduled (loaded)   : {sim['loaded']}")
    print(f"    Actually inserted    : {sim['inserted']}")
    print(f"    Never inserted (backlog) : {sim['backlog']} ({sim['backlog_pct']}% of loaded)")
    print(f"    Still running at cap  : {sim['running_at_end']}")
    print(f"    Teleports (gridlock) : {sim['teleports']}")
    if comp:
        print(f"\n  Fleet composition (measured, entered vehicles)")
        print(f"    HDV : {comp['n_hdv']}   CAV : {comp['n_cav']}   -> total cars : {comp['total_cars']}")
        print(f"    CAV penetration : {comp['cav_pct']}%  (CAV / total cars)")
        print(f"    Bikes : {comp['n_bike']}   Pedestrians : {comp['n_ped']}   (excluded from CAV%)")
    if ray:
        print(f"\n  Ray-casting observability (avg % of objects visible per step)")
        print(f"    All non-CAV : {ray['avg_obs']}%")
        print(f"    Cars (HDV)  : {ray['obs_hdv']}%    Pedestrians : {ray['obs_ped']}%    Cyclists : {ray['obs_bike']}%")
    if perc:
        print(f"\n  CAV-perception accuracy (perceived vs ground truth)")
        print(f"    Mean queue-length error : {perc['q_err']} road users/phase")
        print(f"    Mean waiting-time error : {perc['w_err']} s/phase")
    print()
