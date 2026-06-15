#!/usr/bin/env python3
"""Coverage-path comparison (geometric analysis, no simulation).

Compares the selected route (the actual patrol waypoints from
configs/scenario_nominal.yaml) against standard coverage strategies
generated over the same operational region. The operational region is the
convex hull of the selected route's waypoints (buffered by half a sweep
spacing); baseline strategies are clipped to that region so the comparison
is like-for-like. All metrics are computed geometrically on a grid —
nothing is hardcoded.

This comparison only shows the selected route is reasonable; it does not
claim a new coverage-planning algorithm.

Output: results/csv/coverage_path_comparison.csv
"""
import os
import math
import yaml
import numpy as np
import pandas as pd
from shapely.geometry import Polygon, LineString, MultiPoint, Point

WORKSPACE = "/home/px4_sitl/sim_paper"

# Sensor swath: ground footprint at 60 m altitude with 60 deg FOV half-angle
ALT = 60.0
FOV_HALF_DEG = 60.0
SWATH_HALF = ALT * math.tan(math.radians(FOV_HALF_DEG)) + 20.0  # + event radius

CRUISE_SPEED = 12.0  # m/s mission speed (MissionItem speed_m_s)
GRID = 25.0          # m grid resolution for gap/overlap accounting
SPACING = 300.0      # sweep-line spacing used by the selected route


def selected_route():
    cfg = yaml.safe_load(open(os.path.join(WORKSPACE, "configs/scenario_nominal.yaml")))
    fsm = cfg['fsm_node']['ros__parameters']
    pts = []
    for i in range(1, 7):
        flat = fsm[f'patrol_waypoints_{i}']
        pts += [(flat[j], flat[j + 1]) for j in range(0, len(flat), 3)]
    return pts


def region_polygon(route_pts):
    hull = MultiPoint([Point(n, e) for (n, e) in route_pts]).convex_hull
    return hull.buffer(SPACING / 2.0, join_style=2)


def clipped_sweep_lines(region, axis, spacing=SPACING):
    """Generate boustrophedon sweep legs along `axis` clipped to the region.

    axis='NS': legs run along the north coordinate at constant east;
    axis='EW': legs run along east at constant north.
    Returns an ordered waypoint list with serpentine connections.
    """
    n_min, e_min, n_max, e_max = (region.bounds[0], region.bounds[1],
                                  region.bounds[2], region.bounds[3])
    pts = []
    flip = False
    if axis == 'NS':
        lines = np.arange(e_min + spacing / 2, e_max, spacing)
        for e in lines:
            seg = LineString([(n_min - 10, e), (n_max + 10, e)]).intersection(region)
            if seg.is_empty or seg.length < GRID:
                continue
            cs = list(seg.geoms) if seg.geom_type == 'MultiLineString' else [seg]
            longest = max(cs, key=lambda g: g.length)
            (a, b) = (longest.coords[0], longest.coords[-1])
            leg = [(a[0], e), (b[0], e)] if not flip else [(b[0], e), (a[0], e)]
            pts += leg
            flip = not flip
    else:
        lines = np.arange(n_min + spacing / 2, n_max, spacing)
        for n in lines:
            seg = LineString([(n, e_min - 10), (n, e_max + 10)]).intersection(region)
            if seg.is_empty or seg.length < GRID:
                continue
            cs = list(seg.geoms) if seg.geom_type == 'MultiLineString' else [seg]
            longest = max(cs, key=lambda g: g.length)
            (a, b) = (longest.coords[0], longest.coords[-1])
            leg = [(n, a[1]), (n, b[1])] if not flip else [(n, b[1]), (n, a[1])]
            pts += leg
            flip = not flip
    return pts


def boustrophedon_2cell(region, spacing=SPACING):
    """Exact 2-cell decomposition split at the region's east midline, each
    cell swept NS independently (transit between cells included)."""
    n_min, e_min, n_max, e_max = (region.bounds[0], region.bounds[1],
                                  region.bounds[2], region.bounds[3])
    e_mid = 0.5 * (e_min + e_max)
    pts = []
    for (lo, hi) in [(e_min, e_mid), (e_mid, e_max)]:
        cell = region.intersection(
            Polygon([(n_min - 10, lo), (n_max + 10, lo), (n_max + 10, hi), (n_min - 10, hi)]))
        if cell.is_empty:
            continue
        sub = clipped_sweep_lines(cell, 'NS', spacing)
        pts += sub
    return pts


def spiral_inward(region, spacing=SPACING):
    """Inward spiral: successive inward offsets of the region boundary."""
    pts = []
    poly = region
    while not poly.is_empty and poly.area > spacing * spacing:
        boundary = list(poly.exterior.coords)
        pts += boundary
        poly = poly.buffer(-spacing, join_style=2)
        if poly.geom_type == 'MultiPolygon':
            poly = max(poly.geoms, key=lambda g: g.area)
    return pts


def path_length(pts):
    return sum(math.dist(a, b) for a, b in zip(pts, pts[1:]))


def turn_count(pts, thresh_deg=30.0):
    cnt = 0
    for a, b, c in zip(pts, pts[1:], pts[2:]):
        v1 = (b[0] - a[0], b[1] - a[1])
        v2 = (c[0] - b[0], c[1] - b[1])
        m1, m2 = math.hypot(*v1), math.hypot(*v2)
        if m1 < 1.0 or m2 < 1.0:
            continue
        cosang = max(-1.0, min(1.0, (v1[0] * v2[0] + v1[1] * v2[1]) / (m1 * m2)))
        if math.degrees(math.acos(cosang)) > thresh_deg:
            cnt += 1
    return cnt


def coverage_stats(pts, region):
    """Grid-based gap and overlap accounting inside the region polygon."""
    n_min, e_min, n_max, e_max = (region.bounds[0], region.bounds[1],
                                  region.bounds[2], region.bounds[3])
    ns = np.arange(n_min + GRID / 2, n_max, GRID)
    es = np.arange(e_min + GRID / 2, e_max, GRID)
    nn, ee = np.meshgrid(ns, es, indexing='ij')

    from shapely import vectorized
    try:
        inside = vectorized.contains(region, nn, ee)
    except Exception:
        inside = np.array([[region.contains(Point(n, e)) for e in es] for n in ns])

    passes = np.zeros_like(nn, dtype=int)
    for a, b in zip(pts, pts[1:]):
        seg = np.array(b) - np.array(a)
        seg_len2 = seg @ seg
        if seg_len2 < 1.0:
            continue
        dn = nn - a[0]
        de = ee - a[1]
        t = np.clip((dn * seg[0] + de * seg[1]) / seg_len2, 0.0, 1.0)
        dist = np.hypot(dn - t * seg[0], de - t * seg[1])
        passes += (dist <= SWATH_HALF).astype(int)

    total = int(np.count_nonzero(inside))
    cell_area = (GRID * GRID) / 1e6  # km^2 per grid cell
    gap_pct = 100.0 * np.count_nonzero((passes == 0) & inside) / total
    overlap_pct = 100.0 * np.count_nonzero((passes >= 2) & inside) / total
    fov_coverage_pct = 100.0 * np.count_nonzero((passes >= 1) & inside) / total
    uncovered_area_km2 = np.count_nonzero((passes == 0) & inside) * cell_area
    return gap_pct, overlap_pct, fov_coverage_pct, uncovered_area_km2


def per_uav_lengths(name, pts, route):
    """Per-UAV segment lengths. The selected route has 6 explicit partitions;
    baseline single-paths are split into 6 contiguous equal-count chunks so a
    workload-imbalance comparison is meaningful."""
    if name.startswith("Selected"):
        lens = []
        for i in range(1, 7):
            seg = route_partition(i)
            lens.append(path_length(seg))
        return lens
    # split ordered path into 6 contiguous chunks
    n = max(1, len(pts) // 6)
    chunks = [pts[i:i + n + 1] for i in range(0, len(pts), n)][:6]
    return [path_length(c) for c in chunks if len(c) > 1]


def route_partition(i):
    cfg = yaml.safe_load(open(os.path.join(WORKSPACE, "configs/scenario_nominal.yaml")))
    flat = cfg['fsm_node']['ros__parameters'][f'patrol_waypoints_{i}']
    return [(flat[j], flat[j + 1]) for j in range(0, len(flat), 3)]


def main():
    route = selected_route()
    region = region_polygon(route)
    print(f"Operational region: convex hull of selected route, area = "
          f"{region.area / 1e6:.2f} km^2, swath half-width = {SWATH_HALF:.0f} m")

    strategies = {
        "Selected (orientation-optimized lawnmower, 6-UAV partition)": route,
        "North-South lawnmower": clipped_sweep_lines(region, 'NS'),
        "East-West lawnmower": clipped_sweep_lines(region, 'EW'),
        "Boustrophedon (2-cell)": boustrophedon_2cell(region),
        "Spiral/inward coverage": spiral_inward(region),
    }

    rows = []
    for name, pts in strategies.items():
        L = path_length(pts)
        gap, overlap, fov_cov, uncov = coverage_stats(pts, region)
        uav_lens = per_uav_lengths(name, pts, route)
        mean_uav = float(np.mean(uav_lens)) if uav_lens else float('nan')
        max_uav = float(np.max(uav_lens)) if uav_lens else float('nan')
        # workload imbalance = (max - mean) / mean over per-UAV lengths
        imbalance = (max_uav - mean_uav) / mean_uav if mean_uav > 0 else float('nan')
        rows.append({
            "strategy": name,
            "total_path_length_km": round(L / 1000.0, 2),
            "mean_per_uav_km": round(mean_uav / 1000.0, 2),
            "max_per_uav_km": round(max_uav / 1000.0, 2),
            "fov_coverage_pct": round(fov_cov, 1),
            "uncovered_area_km2": round(uncov, 2),
            "overlap_pct": round(overlap, 1),
            "turn_count": turn_count(pts),
            "waypoint_count": len(pts),
            "est_completion_time_min": round(max_uav / CRUISE_SPEED / 60.0, 1),
            "workload_imbalance": round(imbalance, 3),
        })
        print(rows[-1])

    out = os.path.join(WORKSPACE, "results/csv/coverage_path_comparison.csv")
    os.makedirs(os.path.dirname(out), exist_ok=True)
    pd.DataFrame(rows).to_csv(out, index=False)
    print(f"Saved {out}")


if __name__ == "__main__":
    main()
