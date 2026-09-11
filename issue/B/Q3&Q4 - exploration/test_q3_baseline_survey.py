"""Q3 survey: keep scanning until two bearings are spatially well separated."""
from __future__ import annotations

import math
import statistics

import local_env
import strategy
import geometry as geo

CHANNELS = list(range(1, 21))


def survey_q3_baseline(iface, points, min_baseline, max_signals=6):
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
                order = sorted((c for c in needed if c < cur), reverse=True) + sorted(c for c in needed if c >= cur)
            else:
                order = sorted(c for c in needed if c > cur) + sorted((c for c in needed if c <= cur), reverse=True)
        for ch in order:
            res = iface.measure(p, ch)
            d = data[ch]
            if res.measure_result == "direction":
                d.signals.append((p, res.svd_deg))
                if d.first_signal_pos is None:
                    d.first_signal_pos = p
                if len(d.signals) >= 2:
                    baseline = max(math.dist(a[0], b[0]) for a in d.signals for b in d.signals)
                    if baseline >= min_baseline or len(d.signals) >= max_signals:
                        enough.add(ch)
            elif res.measure_result == "near":
                d.near_points.append(p)
                if d.first_signal_pos is None:
                    d.first_signal_pos = p
                enough.add(ch)
            else:
                d.no_signal_points.append(p)
    return data


def run_case(seed, min_baseline, max_signals):
    env = local_env.LocalEnv("q3", seed=seed)
    env.enter()
    data = survey_q3_baseline(env, strategy.q3_survey_points(), min_baseline, max_signals)
    cleared, failed = strategy.clear_all(env, "q3", data)
    return {
        "virtual_time": env.virtual_time,
        "cleared": cleared,
        "total": len(env.sources),
        "measures": env.measure_count,
        "clears": env.clear_count,
    }


def main():
    for baseline in [300, 500, 700, 900, 1200]:
        for mx in [3, 4, 5, 6]:
            rows = [run_case(1000 + i, baseline, mx) for i in range(30)]
            if any(r["cleared"] != r["total"] for r in rows):
                print("FAIL", baseline, mx, sum(r["total"] - r["cleared"] for r in rows))
                continue
            print("base", baseline, "mx", mx,
                  "mean_v", round(statistics.mean(r["virtual_time"] for r in rows), 1),
                  "mean_meas", round(statistics.mean(r["measures"] for r in rows), 1),
                  "max_v", round(max(r["virtual_time"] for r in rows), 1))


if __name__ == "__main__":
    main()
