"""Test Q3 localization variant: measure at the nearest point of F to the
robot, provided that point is within 1000 m of every vertex of F."""
from __future__ import annotations

import math
import statistics

import local_env
import strategy
import geometry as geo

CLEAR = 20.0
MAX_ITER = 10


def closest_point_on_segment(p, a, b):
    ax, ay = a
    bx, by = b
    dx, dy = bx - ax, by - ay
    if dx == 0 and dy == 0:
        return a
    t = ((p[0] - ax) * dx + (p[1] - ay) * dy) / (dx * dx + dy * dy)
    t = max(0.0, min(1.0, t))
    return (ax + t * dx, ay + t * dy)


def closest_point_in_convex_polygon(p, poly):
    if geo.point_in_convex_polygon(p, poly, eps=1e-8):
        return p
    best = None
    bd = 1e100
    for i in range(len(poly)):
        q = closest_point_on_segment(p, poly[i], poly[(i + 1) % len(poly)])
        d = math.dist(p, q)
        if d < bd:
            bd = d
            best = q
    return best


def clear_channel_q3_near(iface, channel, d):
    if d.near_points:
        p = min(d.near_points, key=lambda q: math.dist(iface.pos, q))
        return strategy._clear_at(iface, p, channel)
    signals = list(d.signals)
    disks = [p for p, _ in signals]
    poly = strategy._rebuild_poly(signals, disks)
    if not poly:
        return False
    for _ in range(MAX_ITER):
        c_near = closest_point_in_convex_polygon(iface.pos, poly)
        r_near = max(math.dist(c_near, v) for v in poly)
        c_mec, r_mec = geo.min_enclosing_circle(poly)
        if r_mec <= CLEAR + 1e-6:
            if strategy._clear_at(iface, c_mec, channel):
                return True
            return strategy._cover_and_clear(iface, poly, channel, signals)
        # Use the nearest point if it is guaranteed to receive signal.
        if r_near <= 1000.0 + 1e-6:
            x = c_near
            r = r_near
        else:
            x = c_mec
            r = r_mec
        if r <= CLEAR + 1e-6:
            if strategy._clear_at(iface, x, channel):
                return True
            return strategy._cover_and_clear(iface, poly, channel, signals)
        if r <= 25.0 and strategy._clear_at(iface, x, channel):
            return True
        res = iface.measure(x, channel)
        if res.measure_result == "near":
            if strategy._clear_at(iface, x, channel):
                return True
            return strategy._cover_and_clear(iface, poly, channel, signals)
        if res.measure_result == "direction":
            signals.append((x, res.svd_deg))
            disks.append(x)
            poly = strategy._rebuild_poly(signals, disks)
            if not poly:
                return False
            continue
        return strategy._cover_and_clear(iface, poly, channel, signals)
    return strategy._cover_and_clear(iface, poly, channel, signals)


def run_case(seed):
    env = local_env.LocalEnv("q3", seed=seed)
    env.enter()
    data = strategy.survey(env, strategy.q3_survey_points(), env.pos, min_signals=2)
    orig = strategy.clear_channel_q3
    strategy.clear_channel_q3 = clear_channel_q3_near
    cleared, failed = strategy.clear_all(env, "q3", data)
    strategy.clear_channel_q3 = orig
    return {
        "virtual_time": env.virtual_time,
        "cleared": cleared,
        "total": len(env.sources),
        "measures": env.measure_count,
        "clears": env.clear_count,
    }


def main():
    rows = [run_case(1000 + i) for i in range(15)]
    print("fail", sum(r["total"] - r["cleared"] for r in rows))
    print("mean_v", statistics.mean(r["virtual_time"] for r in rows))
    print("max_v", max(r["virtual_time"] for r in rows))
    print("mean_meas", statistics.mean(r["measures"] for r in rows))
    print("mean_clear", statistics.mean(r["clears"] for r in rows))


if __name__ == "__main__":
    main()
