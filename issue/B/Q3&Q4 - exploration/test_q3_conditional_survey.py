"""Q3 conditional survey: stop scanning a channel once its feasible MEC radius
is small enough, rather than using a fixed min_signals threshold."""
from __future__ import annotations

import math
import statistics

import local_env
import strategy
import geometry as geo

CHANNELS = list(range(1, 21))


def survey_q3_conditional(iface, points, stop_radius):
    data = {ch: strategy.ChannelData() for ch in CHANNELS}
    enough = set()
    route = geo.order_route(points, iface.pos)
    for p in route:
        cur = iface.current_channel
        needed = [c for c in CHANNELS if c not in enough]
        if not needed:
            continue
        lo, hi = min(needed), max(needed)
        if cur <= lo:
            order = sorted(needed)
        elif cur >= hi:
            order = sorted(needed, reverse=True)
        else:
            left_first = (cur - lo) + (hi - lo)
            right_first = (hi - cur) + (hi - lo)
            if left_first <= right_first:
                order = sorted((c for c in needed if c < cur), reverse=True) + \
                    sorted(c for c in needed if c >= cur)
            else:
                order = sorted(c for c in needed if c > cur) + \
                    sorted((c for c in needed if c <= cur), reverse=True)
        for ch in order:
            res = iface.measure(p, ch)
            d = data[ch]
            if res.measure_result == "direction":
                d.signals.append((p, res.svd_deg))
                if d.first_signal_pos is None:
                    d.first_signal_pos = p
                if len(d.signals) >= 2:
                    poly = strategy._rebuild_poly(d.signals, [x[0] for x in d.signals])
                    if poly:
                        _, r = geo.min_enclosing_circle(poly)
                        if r <= stop_radius + 1e-6:
                            enough.add(ch)
            elif res.measure_result == "near":
                d.near_points.append(p)
                if d.first_signal_pos is None:
                    d.first_signal_pos = p
                enough.add(ch)
            else:
                d.no_signal_points.append(p)
    return data


def run_case(seed, stop_radius):
    env = local_env.LocalEnv("q3", seed=seed)
    env.enter()
    data = survey_q3_conditional(env, strategy.q3_survey_points(), stop_radius)
    cleared, failed = strategy.clear_all(env, "q3", data)
    return {
        "virtual_time": env.virtual_time,
        "cleared": cleared,
        "total": len(env.sources),
        "measures": env.measure_count,
        "clears": env.clear_count,
    }


def main():
    for stop in [20, 40, 60, 80, 100, 150, 200, 300]:
        rows = [run_case(1000 + i, stop) for i in range(50)]
        if any(r["cleared"] != r["total"] for r in rows):
            print("FAIL", stop, sum(r["total"] - r["cleared"] for r in rows))
            continue
        print("stop", stop,
              "mean_v", round(statistics.mean(r["virtual_time"] for r in rows), 1),
              "mean_meas", round(statistics.mean(r["measures"] for r in rows), 1),
              "mean_clear", round(statistics.mean(r["clears"] for r in rows), 1),
              "max_v", round(max(r["virtual_time"] for r in rows), 1))


if __name__ == "__main__":
    main()
