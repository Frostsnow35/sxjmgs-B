"""Monte-Carlo rehearsal of Q3/Q4 strategies on the local emulator.

Usage:
    python monte_carlo_test.py --mode q3 --cases 50 --seed 1
    python monte_carlo_test.py --mode q4 --cases 20 --seed 1
"""
from __future__ import annotations

import argparse
import json
import statistics
import time
from pathlib import Path

import local_env
import strategy


def run_case(mode: str, seed: int) -> dict:
    env = local_env.LocalEnv(mode, seed=seed)
    env.enter()
    t0 = time.perf_counter()
    if mode == "q3":
        res = strategy.run_q3(env)
    else:
        res = strategy.run_q4(env)
    wall = time.perf_counter() - t0
    total = len(env.sources)
    cleared = len(env.cleared_channels)
    return {
        "mode": mode,
        "seed": seed,
        "total": total,
        "cleared": cleared,
        "cleared_ratio": cleared / total if total else 0.0,
        "avg_time": env.virtual_time / cleared if cleared else None,
        "virtual_time": env.virtual_time,
        "wall_seconds": wall,
        "requests": env.requests,
        "measures": env.measure_count,
        "clears": env.clear_count,
        "active_channels": res["active_channels"],
        "failed": res["failed"],
        "sources": env.source_state(),
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mode", choices=["q3", "q4"], default="q3")
    ap.add_argument("--cases", type=int, default=50)
    ap.add_argument("--seed", type=int, default=1)
    args = ap.parse_args()

    results = [run_case(args.mode, args.seed + i) for i in range(args.cases)]
    times = [r["avg_time"] for r in results if r["avg_time"] is not None]
    ratios = [r["cleared_ratio"] for r in results]
    print(f"mode={args.mode} cases={args.cases}")
    print(f"cleared ratio: min={min(ratios):.3f} mean={statistics.mean(ratios):.3f}")
    print(f"avg clear time: mean={statistics.mean(times):.2f}s "
          f"median={statistics.median(times):.2f}s "
          f"min={min(times):.2f}s max={max(times):.2f}s")
    print(f"virtual time: mean={statistics.mean(r['virtual_time'] for r in results):.2f}s")
    print(f"requests: mean={statistics.mean(r['requests'] for r in results):.1f} "
          f"max={max(r['requests'] for r in results)}")
    print(f"failures: {sum(r['failed'] for r in results)}")
    out = Path(f"monte_carlo_{args.mode}.json")
    out.write_text(json.dumps(results, indent=2), encoding="utf-8")
    print("wrote", out)


if __name__ == "__main__":
    main()
