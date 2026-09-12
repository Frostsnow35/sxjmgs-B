"""Run the reviewable Q30912 solver for three consecutive practice rounds.

Usage:
    python run_q30912_reviewable_3x.py --robot-id <team_id>

The user starts each Q3 practice round manually in the emulator.  This wrapper
waits until the emulator interface is open, runs the solver once, then waits
for the next round.  It never alters the solver algorithm.
"""
from __future__ import annotations

import argparse
import json
import subprocess
import sys
import time
from pathlib import Path

BASE = Path(__file__).resolve().parent
SOLVER = BASE / "B3_robot_solver_v6_reviewable.py"


def log_has_exit(path: Path) -> bool:
    if not path.exists():
        return False
    for line in path.read_text(encoding="utf-8").splitlines():
        if not line.strip():
            continue
        obj = json.loads(line)
        if obj.get("path") == "/exit" and obj.get("response", {}).get("accepted") is True:
            return True
    return False


def run_round(robot_id: str, round_no: int, base_url: str):
    log = BASE / f"B3_review_round{round_no:02d}_{time.strftime('%H%M%S')}.jsonl"
    cmd = [
        sys.executable, str(SOLVER),
        "--robot-id", robot_id,
        "--base-url", base_url,
        "--log", str(log),
    ]
    attempts = 0
    while attempts < 600:
        attempts += 1
        print(f"[round {round_no:02d}] attempt {attempts}: launching solver", flush=True)
        proc = subprocess.run(cmd, cwd=str(BASE), timeout=180)
        if log_has_exit(log):
            print(f"[round {round_no:02d}] completed", flush=True)
            return log
        if proc.returncode != 0:
            print(f"[round {round_no:02d}] solver exited rc={proc.returncode}; "
                  "waiting for next test window", flush=True)
        time.sleep(1.0)
    raise TimeoutError(f"round {round_no} did not complete")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--robot-id", required=True)
    ap.add_argument("--base-url", default="http://127.0.0.1:2026")
    ap.add_argument("--rounds", type=int, default=3)
    args = ap.parse_args()

    for i in range(1, args.rounds + 1):
        log = run_round(args.robot_id, i, args.base_url)
        print(f"[summary] round {i}: log={log.name}", flush=True)
    print("ALL_ROUNDS_DONE", flush=True)


if __name__ == "__main__":
    main()
