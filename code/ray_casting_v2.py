# -*- coding: utf-8 -*-
"""
ray_casting_v2.py
=================
Per-CAV ray casting sensor module.

Key design decisions (all research-motivated):

1.  Each CAV casts its own 72 rays from its OWN position.
    This reflects reality: a CAV's onboard LiDAR/radar is mounted on the
    vehicle, not at the roadside. Its field of view is centred on itself.

2.  CAV -> CAV hits are DISCARDED.
    CAVs share position via V2X (Vehicle-to-Everything) communication, so
    physically "seeing" another CAV with a ray adds zero new information.
    Only hits on HDVs, pedestrians, and cyclists are meaningful detections.
    This is the core of Research Question 3: what can CAVs observe that the
    network would otherwise be blind to?

3.  Multi-ray hits on the same object are deduplicated.
    A wide vehicle may be struck by 3-4 adjacent rays. We count it as ONE
    detected object and record the closest-ray distance as the range estimate.

4.  Fleet-level aggregation.
    After all CAVs scan, results are merged into a single fleet view that
    answers: "what does the collective CAV fleet know right now, beyond
    what V2X already tells them?"

Output at each simulation step:
    - Per-CAV detection list (which non-CAV objects each CAV can see)
    - Fleet summary: unique non-CAV objects detected, counts by type
    - Blind-spot objects: HDVs/peds/bikes that NO CAV detected
      (these are the true unknowns the controller must estimate)
"""

import math


# ---------------------------------------------------------------------------
# Single-ray geometry helper (unchanged from v1)
# ---------------------------------------------------------------------------
def _ray_circle_hit(ox, oy, dx, dy, cx, cy, radius, max_range):
    """
    Returns distance t along ray (ox,oy)+(dx,dy)*t to nearest surface of a
    circle of given radius centred at (cx,cy), or None if no hit within
    max_range. (dx,dy) must be a unit vector.
    """
    fx, fy = cx - ox, cy - oy
    t_proj = fx * dx + fy * dy
    perp_sq = fx*fx + fy*fy - t_proj*t_proj
    r_sq = radius * radius
    if perp_sq > r_sq:
        return None
    half_chord = math.sqrt(max(r_sq - perp_sq, 0.0))
    t_hit = t_proj - half_chord
    if t_hit < 0:
        t_hit = t_proj + half_chord
    return t_hit if 0 <= t_hit <= max_range else None


# ---------------------------------------------------------------------------
# Per-CAV sensor
# ---------------------------------------------------------------------------
class CAVSensor:
    """
    Represents the onboard sensor of a single CAV.
    Instantiated fresh each simulation step (positions change every step).
    """

    NON_INFORMATIVE_TYPES = {"CAV"}   # hits on these are discarded

    def __init__(self, cav_id, x, y,
                 max_range=30.0, angle_step_deg=5.0, obstacle_radius=0.9):
        self.cav_id        = cav_id
        self.x             = x
        self.y             = y
        self.max_range     = max_range
        self.angle_step    = angle_step_deg
        self.obs_radius    = obstacle_radius
        self.num_rays      = int(360 / angle_step_deg)
        # Precompute unit direction vectors
        self._dirs = [
            (math.cos(math.radians(i * angle_step_deg)),
             math.sin(math.radians(i * angle_step_deg)))
            for i in range(self.num_rays)
        ]

    def scan(self, all_obstacles):
        """
        Cast all rays against the obstacle list.

        all_obstacles : list of dicts with keys id, type, x, y
                        (includes this CAV itself and all other CAVs)

        Returns: list of detection dicts, one per unique non-CAV object hit.
        Each dict:
            cav_id       : which CAV made this detection
            object_id    : id of the detected object
            object_type  : HDV / bike_profile / pedestrian
            distance_m   : closest-ray distance to object surface (metres)
            rays_hit     : how many of the 72 rays struck this object
            bearing_deg  : angle of the closest-hit ray (0=East, CCW)
        """
        # Map object_id -> best (distance, ray_angle, rays_hit_count)
        best_hits = {}  # object_id -> [min_dist, bearing_deg, hit_count]

        # ── Occlusion model ──────────────────────────────────────────────────
        # A real LiDAR/radar ray stops at the FIRST physical surface it meets;
        # objects behind that surface are hidden.  So for each ray we compute
        # the hit distance to EVERY physical obstacle (other CAVs included --
        # a CAV body blocks the beam just like any vehicle), keep only the
        # NEAREST hit, and register a detection only if that nearest object is
        # a non-CAV.  If a CAV is nearest, the ray is consumed by it and sees
        # nothing usable along that bearing -- but that is fine, because the
        # blocking CAV casts its own rays and shares state over V2X, so the
        # fleet loses no information (this is why CAV->CAV blocking is not a
        # problem, exactly as intended).
        # ── Spatial pre-filter (big speed win, identical results) ────────────
        # An obstacle further away than max_range can never be hit by any ray,
        # so screen the list ONCE per CAV instead of re-testing every obstacle
        # inside all 72 ray loops.  Uses a cheap squared-distance bounding test.
        reach_sq = (self.max_range + self.obs_radius) ** 2
        near = []
        for obs in all_obstacles:
            if obs['id'] == self.cav_id:
                continue
            ddx = obs['x'] - self.x
            ddy = obs['y'] - self.y
            if ddx*ddx + ddy*ddy <= reach_sq:
                near.append(obs)

        for ray_idx, (dx, dy) in enumerate(self._dirs):
            angle_deg = ray_idx * self.angle_step

            nearest_dist = None
            nearest_obs  = None
            for obs in near:
                dist = _ray_circle_hit(
                    self.x, self.y, dx, dy,
                    obs['x'], obs['y'], self.obs_radius, self.max_range
                )
                if dist is None:
                    continue
                if nearest_dist is None or dist < nearest_dist:
                    nearest_dist = dist
                    nearest_obs  = obs

            # Nothing in this ray's path, or the nearest surface is a CAV
            # (beam consumed, no new info) -> ray yields no detection.
            if nearest_obs is None:
                continue
            if nearest_obs['type'] in self.NON_INFORMATIVE_TYPES:
                continue

            oid = nearest_obs['id']
            if oid not in best_hits:
                best_hits[oid] = {
                    'object_type': nearest_obs['type'],
                    'min_dist':    nearest_dist,
                    'bearing_deg': angle_deg,
                    'hit_count':   1,
                }
            else:
                best_hits[oid]['hit_count'] += 1
                if nearest_dist < best_hits[oid]['min_dist']:
                    best_hits[oid]['min_dist']    = nearest_dist
                    best_hits[oid]['bearing_deg'] = angle_deg

        # Build clean detection list
        detections = []
        for oid, info in best_hits.items():
            detections.append({
                'cav_id':      self.cav_id,
                'object_id':   oid,
                'object_type': info['object_type'],
                'distance_m':  round(info['min_dist'], 2),
                'rays_hit':    info['hit_count'],
                'bearing_deg': round(info['bearing_deg'], 1),
            })
        # Sort by distance (nearest first)
        detections.sort(key=lambda d: d['distance_m'])
        return detections


# ---------------------------------------------------------------------------
# Fleet-level aggregator
# ---------------------------------------------------------------------------
class FleetPerceptionAggregator:
    """
    Runs per-CAV scans for every CAV on the map and merges results into a
    single fleet-wide picture of what the CAV fleet collectively detects.
    """

    def __init__(self, max_range=30.0, angle_step_deg=5.0, obstacle_radius=0.9):
        self.max_range     = max_range
        self.angle_step    = angle_step_deg
        self.obs_radius    = obstacle_radius

    def scan_fleet(self, all_obstacles):
        """
        all_obstacles : list of dicts {id, type, x, y} for ALL objects on map
                        (vehicles of any type + pedestrians)

        Returns a FleetScanResult.
        """
        cav_list = [o for o in all_obstacles if o['type'] == 'CAV']

        per_cav_results = {}
        fleet_detections = {}  # object_id -> best detection across all CAVs

        for cav in cav_list:
            sensor = CAVSensor(
                cav['id'], cav['x'], cav['y'],
                self.max_range, self.angle_step, self.obs_radius
            )
            dets = sensor.scan(all_obstacles)
            per_cav_results[cav['id']] = dets

            for det in dets:
                oid = det['object_id']
                if oid not in fleet_detections:
                    fleet_detections[oid] = det.copy()
                    fleet_detections[oid]['detected_by'] = [cav['id']]
                else:
                    fleet_detections[oid]['detected_by'].append(cav['id'])
                    # Keep the closest detection
                    if det['distance_m'] < fleet_detections[oid]['distance_m']:
                        fleet_detections[oid]['distance_m'] = det['distance_m']
                        fleet_detections[oid]['bearing_deg'] = det['bearing_deg']
                        fleet_detections[oid]['rays_hit']    = det['rays_hit']

        # Find blind-spot objects: non-CAV objects that NO CAV detected
        non_cav_ids = {o['id'] for o in all_obstacles if o['type'] != 'CAV'}
        detected_ids = set(fleet_detections.keys())
        blind_spot_ids = non_cav_ids - detected_ids
        blind_spots = [o for o in all_obstacles if o['id'] in blind_spot_ids]

        return FleetScanResult(
            n_cavs          = len(cav_list),
            per_cav         = per_cav_results,
            fleet_detections= fleet_detections,
            blind_spots     = blind_spots,
            all_obstacles   = all_obstacles,
        )


# ---------------------------------------------------------------------------
# Result container + pretty printer
# ---------------------------------------------------------------------------
class FleetScanResult:
    def __init__(self, n_cavs, per_cav, fleet_detections,
                 blind_spots, all_obstacles):
        self.n_cavs            = n_cavs
        self.per_cav           = per_cav           # dict cav_id -> [dets]
        self.fleet_detections  = fleet_detections  # dict obj_id -> det
        self.blind_spots       = blind_spots        # list of undetected objs
        self.all_obstacles     = all_obstacles

    # -- Convenience counts --------------------------------------------------
    def n_detected(self, obj_type=None):
        if obj_type is None:
            return len(self.fleet_detections)
        return sum(1 for d in self.fleet_detections.values()
                   if d['object_type'] == obj_type)

    def n_blind(self, obj_type=None):
        if obj_type is None:
            return len(self.blind_spots)
        return sum(1 for o in self.blind_spots if o['type'] == obj_type)

    def total_non_cav(self, obj_type=None):
        if obj_type is None:
            return sum(1 for o in self.all_obstacles if o['type'] != 'CAV')
        return sum(1 for o in self.all_obstacles if o['type'] == obj_type)

    def observability_pct(self, obj_type=None):
        """
        Fraction of objects the fleet can see this step (0-100).

        obj_type=None -> all non-CAV objects combined.
        obj_type='HDV' / 'pedestrian' / 'bike_profile' -> that type only.

        Returns None when that type has zero objects present this step, so the
        metric never reports a misleading 100% or 0% for an absent population
        (e.g. HDVs at 100% CAV penetration).
        """
        total = self.total_non_cav(obj_type)
        if total == 0:
            return None
        detected = self.n_detected(obj_type)
        return round(100.0 * detected / total, 1)

    # -- Summary dict (used for CSV logging) ---------------------------------
    def summary(self):
        return {
            'n_cavs_active':         self.n_cavs,
            'non_cav_objects_total': self.total_non_cav(),
            'fleet_detected_total':  self.n_detected(),
            'detected_HDV':          self.n_detected('HDV'),
            'detected_pedestrian':   self.n_detected('pedestrian'),
            'detected_bike':         self.n_detected('bike_profile'),
            'blind_spot_total':      self.n_blind(),
            'blind_HDV':             self.n_blind('HDV'),
            'blind_pedestrian':      self.n_blind('pedestrian'),
            'blind_bike':            self.n_blind('bike_profile'),
            'observability_pct':      self.observability_pct(),            # all non-CAV
            'observability_pct_HDV':  self.observability_pct('HDV'),
            'observability_pct_ped':  self.observability_pct('pedestrian'),
            'observability_pct_bike': self.observability_pct('bike_profile'),
            'total_HDV':              self.total_non_cav('HDV'),
            'total_pedestrian':       self.total_non_cav('pedestrian'),
            'total_bike':             self.total_non_cav('bike_profile'),
        }

    # -- Human-readable print ------------------------------------------------
    def print_summary(self, sim_time=None):
        tag = f"t={sim_time:4d}s" if sim_time is not None else "t=???s"
        s = self.summary()
        print(
            f"  {tag} | CAVs={s['n_cavs_active']} "
            f"| Detected: HDV={s['detected_HDV']} "
            f"ped={s['detected_pedestrian']} "
            f"bike={s['detected_bike']} "
            f"| BlindSpots: HDV={s['blind_HDV']} "
            f"ped={s['blind_pedestrian']} "
            f"bike={s['blind_bike']} "
            f"| Observability={s['observability_pct']}%"
        )

    def print_per_cav(self, sim_time=None):
        tag = f"t={sim_time}s" if sim_time is not None else ""
        print(f"\n  -- Per-CAV detections {tag} --")
        if not self.per_cav:
            print("  (no CAVs active)")
            return
        for cav_id, dets in self.per_cav.items():
            if not dets:
                print(f"    {cav_id}: nothing detected within range")
            else:
                lines = []
                for d in dets:
                    lines.append(
                        f"{d['object_type']}:{d['object_id']} "
                        f"@ {d['distance_m']}m "
                        f"({d['rays_hit']} rays, bearing {d['bearing_deg']}°)"
                    )
                print(f"    {cav_id}: {' | '.join(lines)}")


# ---------------------------------------------------------------------------
# Standalone self-test (python ray_casting_v2.py)
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    print("=" * 65)
    print("Ray Casting v2 — Self Test (no SUMO needed)")
    print("=" * 65)

    # Simulate a small scene:
    #   2 CAVs driving North-South, 3 HDVs driving West-East,
    #   1 pedestrian, 1 cyclist
    test_scene = [
        # CAVs
        {'id': 'CAV_0', 'type': 'CAV', 'x':  1.0, 'y': 20.0},
        {'id': 'CAV_1', 'type': 'CAV', 'x':  1.0, 'y': -5.0},
        # HDVs
        {'id': 'HDV_0', 'type': 'HDV', 'x': -18.0, 'y': 10.0},
        {'id': 'HDV_1', 'type': 'HDV', 'x':  12.0, 'y': 11.0},
        {'id': 'HDV_2', 'type': 'HDV', 'x': 200.0, 'y': 10.0},  # out of range
        # Pedestrian
        {'id': 'ped_0', 'type': 'pedestrian', 'x': 3.0, 'y': 15.0},
        # Cyclist
        {'id': 'bike_0', 'type': 'bike_profile', 'x': -5.0, 'y': 12.0},
    ]

    aggregator = FleetPerceptionAggregator(
        max_range=30.0, angle_step_deg=5.0, obstacle_radius=0.9
    )
    result = aggregator.scan_fleet(test_scene)

    print(f"\nScene: {len(test_scene)} total objects | "
          f"{result.n_cavs} CAVs | "
          f"{result.total_non_cav()} non-CAV objects\n")

    result.print_per_cav(sim_time=0)
    print()
    result.print_summary(sim_time=0)

    print("\nFull fleet detection details:")
    for oid, det in result.fleet_detections.items():
        print(f"  {det['object_type']:12s} id={oid:8s} "
              f"dist={det['distance_m']:5.1f}m  "
              f"rays={det['rays_hit']:2d}  "
              f"detected_by={det['detected_by']}")

    if result.blind_spots:
        print(f"\nBlind spots ({len(result.blind_spots)} objects NO CAV detected):")
        for o in result.blind_spots:
            print(f"  {o['type']} id={o['id']} at ({o['x']}, {o['y']})")
    else:
        print("\nNo blind spots — all non-CAV objects detected by at least one CAV.")

    print("\nSummary dict:", result.summary())
