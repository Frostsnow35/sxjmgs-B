"""Paired LocalEnv comparison for the Q4 joint-projection action hint.

This script never creates a network client.  It runs the deterministic local
emulator twice on every seed: the current strategy and a control that disables
only the optional joint-projection summary.  Conservative safe-negative
candidate removal remains enabled in both arms.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import statistics

import local_env
import strategy


def run_case(seed: int, enabled: bool) -> dict[str, int | float | bool]:
    """Run one fixed local scenario and return non-sensitive summary fields."""

    calls = 0
    nonempty = 0
    original = strategy._joint_projection_summary

    def counted_summary(poly, signals, negatives):
        nonlocal calls, nonempty
        calls += 1
        result = original(poly, signals, negatives)
        if result is not None:
            nonempty += 1
        return result

    if enabled:
        strategy._joint_projection_summary = counted_summary
    else:
        strategy._joint_projection_summary = lambda *_: None
    try:
        env = local_env.LocalEnv("q4", seed=seed)
        env.enter()
        outcome = strategy.run_q4(env)
        return {
            "seed": seed,
            "joint_enabled": enabled,
            "sources": len(env.sources),
            "cleared": outcome["cleared"],
            "failed": outcome["failed"],
            "virtual_time_s": env.virtual_time,
            "measure_actions": env.measure_count,
            "clear_actions": env.clear_count,
            "projection_calls": calls,
            "projection_nonempty": nonempty,
        }
    finally:
        strategy._joint_projection_summary = original


def mean(rows: list[dict], key: str) -> float:
    """Return the arithmetic mean of one numeric result field."""

    return float(statistics.mean(float(row[key]) for row in rows))


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--cases", type=int, default=100)
    parser.add_argument("--seed", type=int, default=2000)
    parser.add_argument("--output", type=Path, default=Path("joint_projection_localenv.json"))
    args = parser.parse_args()
    if args.cases <= 0:
        raise ValueError("--cases must be positive")

    control = [run_case(args.seed + offset, False) for offset in range(args.cases)]
    treatment = [run_case(args.seed + offset, True) for offset in range(args.cases)]
    paired = [
        {
            "seed": base["seed"],
            "sources": base["sources"],
            "control_cleared": base["cleared"],
            "treatment_cleared": improved["cleared"],
            "control_virtual_time_s": base["virtual_time_s"],
            "treatment_virtual_time_s": improved["virtual_time_s"],
            "time_delta_s": float(improved["virtual_time_s"]) - float(base["virtual_time_s"]),
            "control_measures": base["measure_actions"],
            "treatment_measures": improved["measure_actions"],
            "control_clears": base["clear_actions"],
            "treatment_clears": improved["clear_actions"],
            "projection_calls": improved["projection_calls"],
            "projection_nonempty": improved["projection_nonempty"],
        }
        for base, improved in zip(control, treatment, strict=True)
    ]
    payload = {
        "scope": "LocalEnv only; not a simulator drill or formal test",
        "seed_start": args.seed,
        "cases": args.cases,
        "summary": {
            "control_cleared_total": sum(int(row["cleared"]) for row in control),
            "treatment_cleared_total": sum(int(row["cleared"]) for row in treatment),
            "control_time_mean_s": mean(control, "virtual_time_s"),
            "treatment_time_mean_s": mean(treatment, "virtual_time_s"),
            "time_delta_mean_s": mean(paired, "time_delta_s"),
            "control_measure_mean": mean(control, "measure_actions"),
            "treatment_measure_mean": mean(treatment, "measure_actions"),
            "control_clear_mean": mean(control, "clear_actions"),
            "treatment_clear_mean": mean(treatment, "clear_actions"),
            "projection_calls_mean": mean(treatment, "projection_calls"),
            "projection_nonempty_mean": mean(treatment, "projection_nonempty"),
        },
        "paired_cases": paired,
    }
    args.output.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(payload["summary"], ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
