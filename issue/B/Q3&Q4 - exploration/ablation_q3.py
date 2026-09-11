"""Q3 tuning ablation: min_signals policy x safe-negative constraints."""
from __future__ import annotations

import statistics

import local_env
import strategy


def run_case(seed, min_signals, use_safe_neg):
    env = local_env.LocalEnv("q3", seed=seed)
    env.enter()
    old_flag = strategy.Q3_USE_SAFE_NEGATIVES
    strategy.Q3_USE_SAFE_NEGATIVES = use_safe_neg
    try:
        data = strategy.survey(env, strategy.q3_survey_points(), env.pos,
                               min_signals=min_signals)
        cleared, failed = strategy.clear_all(env, "q3", data)
    finally:
        strategy.Q3_USE_SAFE_NEGATIVES = old_flag
    return {
        "seed": seed,
        "virtual_time": env.virtual_time,
        "cleared": cleared,
        "failed": failed,
        "measures": env.measure_count,
        "clears": env.clear_count,
        "total": len(env.sources),
    }


def main():
    for use_safe_neg in (False, True):
        for ms in (1, 2, 3):
            rows = [run_case(1000 + i, ms, use_safe_neg) for i in range(100)]
            if any(r["cleared"] != r["total"] for r in rows):
                print("FAIL", "safe", use_safe_neg, "ms", ms,
                      sum(r["total"] - r["cleared"] for r in rows))
                continue
            print(
                f"safe={int(use_safe_neg)} ms={ms} "
                f"mean_v={statistics.mean(r['virtual_time'] for r in rows):.1f} "
                f"mean_avg={statistics.mean(r['virtual_time'] / r['cleared'] for r in rows):.1f} "
                f"max_avg={max(r['virtual_time'] / r['cleared'] for r in rows):.1f} "
                f"meas={statistics.mean(r['measures'] for r in rows):.1f} "
                f"clear_req={statistics.mean(r['clears'] for r in rows):.1f} "
                f"max_req={max(r['measures'] + r['clears'] for r in rows)}"
            )


if __name__ == "__main__":
    main()
