"""Run 10 Q3 practice tests, then 10 Q4 practice tests.

The user clicks 'start' for each test in the official emulator.  This program
waits for the next interface, runs the strategy, exits and continues.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

from run_drill_batch import run_one_round


def run_sequence(base_url: str, team_id: str, mode: str, rounds: int,
                 enter_wait: float):
    out_path = Path(f"drill_batch_{mode}.jsonl")
    print(f"=== starting {mode} x {rounds} ===", flush=True)
    with out_path.open("a", encoding="utf-8") as out:
        for i in range(1, rounds + 1):
            rec = run_one_round(base_url, team_id, mode, i, enter_wait)
            out.write(json.dumps(rec, ensure_ascii=False) + "\n")
            out.flush()
    print(f"=== finished {mode} x {rounds} ===", flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--team", required=True)
    ap.add_argument("--rounds", type=int, default=10)
    ap.add_argument("--base", default="http://127.0.0.1:2026")
    ap.add_argument("--enter-wait", type=float, default=7200.0)
    args = ap.parse_args()

    run_sequence(args.base, args.team, "q3", args.rounds, args.enter_wait)
    run_sequence(args.base, args.team, "q4", args.rounds, args.enter_wait)
    print("ALL DONE", flush=True)


if __name__ == "__main__":
    main()
