"""Test clearing channels during the Q3 survey when their feasible disk is
already small and the clear point is close to the current survey point."""
from __future__ import annotations

import math
import statistics

import local_env
import strategy
import geometry as geo

CHANNELS = strategy.CHANNELS


def run_online(seed, clear_radius, detour_limit):
    a = 1124.0
    pts = strategy.q3_survey_points(a)
    route = list(pts)
    env = local_env.LocalEnv("q3", seed=seed)
    env.enter()
    data = {ch: strategy.ChannelData() for ch in CHANNELS}
    enough = set()
    cleared = set()
    # Survey with online clear attempts.
    for idx, p in enumerate(route):
        cur = env.current_channel
        needed = [c for c in CHANNELS if c not in enough and c not in cleared]
        lo, hi = min(needed), max(needed)
        if cur <= lo:
            order = sorted(needed)
        elif cur >= hi:
            order = sorted(needed, reverse=True)
        else:
            lf = (cur - lo) + (hi - lo)
            rf = (hi - cur) + (hi - lo)
            order = (sorted((c for c in needed if c < cur), reverse=True) +
                     sorted(c for c in needed if c >= cur)) if lf <= rf else (
                sorted(c for c in needed if c > cur) +
                sorted((c for c in needed if c <= cur), reverse=True))
        for ch in order:
            res = env.measure(p, ch)
            d = data[ch]
            if res.measure_result == "direction":
                d.signals.append((p, res.svd_deg))
                if len(d.signals) >= 2:
                    enough.add(ch)
            elif res.measure_result == "near":
                d.near_points.append(p)
                enough.add(ch)
            else:
                d.no_signal_points.append(p)
        # Online clear candidates: channels not yet cleared with a small F.
        for ch in list(enough):
            d = data[ch]
            if ch in cleared or not d.signals:
                continue
            poly = strategy._rebuild_poly(d.signals, [x[0] for x in d.signals])
            if not poly:
                continue
            c, r = geo.min_enclosing_circle(poly)
            if r <= clear_radius and math.dist(env.pos, c) <= detour_limit:
                if strategy._clear_at(env, c, ch):
                    cleared.add(ch)
    # Finish remaining channels normally.
    for ch in cleared:
        # Remove them from normal clear_all input by pretending no signal?
        data[ch] = strategy.ChannelData()
    c2, f2 = strategy.clear_all(env, "q3", data)
    return env.virtual_time, len(cleared) + c2, len(env.sources)


def main():
    for clear_radius in [40, 60, 80, 120]:
        for detour in [200, 400, 700, 1000]:
            rows = [run_online(1000 + i, clear_radius, detour) for i in range(20)]
            print("R", clear_radius, "det", detour,
                  "fail", sum(1 for r in rows if r[1] != r[2]),
                  "mean", round(statistics.mean(r[0] for r in rows), 1),
                  "max", round(max(r[0] for r in rows), 1))


if __name__ == "__main__":
    main()
