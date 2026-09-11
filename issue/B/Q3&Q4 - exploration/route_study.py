"""Survey route and channel-scan optimisation study.

Produces `route_optimization_study.json` used in the paper draft.
"""
from __future__ import annotations

import json
import math
from pathlib import Path

import strategy
import geometry as geo

SPEED = 5.0
MEASURE_TIME = 5.0
SWITCH_TIME = 1.0
CHANNELS = list(range(1, 21))


def row_sweep_route(points):
    """Simple y-major serpentine route (baseline)."""
    pts = sorted(points, key=lambda p: (round(p[1], 6), p[0]))
    route = []
    rows: dict[float, list[tuple[float, float]]] = {}
    for p in pts:
        rows.setdefault(round(p[1], 6), []).append(p)
    reverse = False
    for y in sorted(rows):
        row = sorted(rows[y])
        if reverse:
            row = row[::-1]
        route.extend(row)
        reverse = not reverse
    return route


def prim_mst(nodes):
    """Prim MST on the node set; a lower bound for a shortest path through all."""
    n = len(nodes)
    if n <= 1:
        return 0.0
    used = [False] * n
    best = [float("inf")] * n
    best[0] = 0.0
    total = 0.0
    for _ in range(n):
        u = min((i for i in range(n) if not used[i]), key=lambda i: best[i])
        used[u] = True
        total += best[u]
        for v in range(n):
            if not used[v]:
                d = math.dist(nodes[u], nodes[v])
                if d < best[v]:
                    best[v] = d
    return total


def study(mode: str, points, start=(0.0, 0.0)):
    optimized = geo.order_route(points, start)
    opt_len = geo.route_length(optimized, start)
    sweep = row_sweep_route(points)
    sweep_len = geo.route_length(sweep, start)
    nodes = [start] + [p for p in points if math.dist(p, start) > 1e-9]
    mst = prim_mst(nodes)
    n = len(points)
    travel_opt = opt_len / SPEED
    detect = n * len(CHANNELS) * MEASURE_TIME
    switch = n * (len(CHANNELS) - 1) * SWITCH_TIME
    survey_opt = travel_opt + detect + switch
    travel_sweep = sweep_len / SPEED
    survey_sweep = travel_sweep + detect + switch
    return {
        "mode": mode,
        "survey_points": n,
        "optimized_route_m": opt_len,
        "row_sweep_route_m": sweep_len,
        "saving_vs_rowsweep_m": sweep_len - opt_len,
        "mst_lower_bound_m": mst,
        "optimized_travel_time_s": travel_opt,
        "detect_time_s": detect,
        "switch_time_s": switch,
        "survey_virtual_time_optimized_s": survey_opt,
        "survey_virtual_time_rowsweep_s": survey_sweep,
        "switch_count_per_point": len(CHANNELS) - 1,
    }


def main():
    q3 = study("q3", strategy.q3_survey_points())
    q4 = study("q4", strategy.q4_survey_points())
    out = {"q3": q3, "q4": q4}
    path = Path("route_optimization_study.json")
    path.write_text(json.dumps(out, indent=2), encoding="utf-8")
    print(json.dumps(out, indent=2))
    print("wrote", path)


if __name__ == "__main__":
    main()
