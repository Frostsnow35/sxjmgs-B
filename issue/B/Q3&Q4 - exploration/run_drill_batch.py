"""Run several practice tests back-to-back.

The user starts each practice test in the official emulator manually.  This
program waits for the next test interface to open, enters, runs the strategy,
exits, saves a separate per-round log and then waits for the next click.

Usage:
    python run_drill_batch.py --mode q3 --rounds 10 --team <team_id>
"""
from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import robot_client
import strategy


def run_one_round(base_url: str, team_id: str, mode: str, round_no: int,
                  enter_wait: float):
    log_path = Path(f"robot_{mode}_round{round_no:02d}.jsonl")
    client = robot_client.OfficialClient(base_url, team_id, mode, log_path=log_path)
    try:
        print(f"[round {round_no:02d}] waiting for interface ...", flush=True)
        enter = client.enter(wait=True, max_wait_s=enter_wait)
        if enter.get("accepted") is not True:
            return {"round": round_no, "ok": False, "reason": "enter not accepted"}
        print(f"[round {round_no:02d}] entered, remaining_real="
              f"{enter.get('remaining_real_duration_s')}s", flush=True)

        if mode == "q3":
            result = strategy.run_q3(client)
        else:
            result = strategy.run_q4(client)

        client.exit()
        # Count accepted actions from the round log.
        n_measure = 0
        n_clear = 0
        n_clear_success = 0
        if log_path.exists():
            with log_path.open("r", encoding="utf-8") as f:
                for line in f:
                    line = line.strip()
                    if not line:
                        continue
                    obj = json.loads(line)
                    if obj.get("type") != "response":
                        continue
                    r = obj.get("response", {})
                    if r.get("accepted") is not True:
                        continue
                    if obj.get("path") == "/measure":
                        n_measure += 1
                    elif obj.get("path") == "/clear":
                        n_clear += 1
                        if r.get("clear_result") == "success":
                            n_clear_success += 1
        vt = client.virtual_time
        avg = vt / n_clear_success if n_clear_success else None
        if avg is not None:
            print(f"[round {round_no:02d}] cleared={result['cleared']} "
                  f"failed={result['failed']} virtual_time={vt:.3f}s "
                  f"avg={avg:.3f}s", flush=True)
        else:
            print(f"[round {round_no:02d}] cleared={result['cleared']} "
                  f"failed={result['failed']} virtual_time={vt:.3f}s",
                  flush=True)
        return {
            "round": round_no,
            "ok": True,
            "cleared": result["cleared"],
            "failed": result["failed"],
            "active_channels": result["active_channels"],
            "virtual_time": vt,
            "avg_time": avg,
            "measure_actions": n_measure,
            "clear_actions": n_clear,
            "clear_success": n_clear_success,
            "log": str(log_path),
        }
    except Exception as e:  # keep waiting for the next round
        print(f"[round {round_no:02d}] ERROR: {e!r}", flush=True)
        try:
            if client.entered:
                client.exit()
        except Exception:
            pass
        return {"round": round_no, "ok": False, "reason": repr(e)}
    finally:
        client.close()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mode", choices=["q3", "q4"], required=True)
    ap.add_argument("--team", required=True)
    ap.add_argument("--rounds", type=int, default=10)
    ap.add_argument("--base", default=robot_client.BASE_URL)
    ap.add_argument("--enter-wait", type=float, default=7200.0)
    args = ap.parse_args()

    out_path = Path(f"drill_batch_{args.mode}.jsonl")
    print(f"batch mode={args.mode} rounds={args.rounds}", flush=True)
    with out_path.open("a", encoding="utf-8") as out:
        for i in range(1, args.rounds + 1):
            rec = run_one_round(args.base, args.team, args.mode, i, args.enter_wait)
            out.write(json.dumps(rec, ensure_ascii=False) + "\n")
            out.flush()


if __name__ == "__main__":
    main()
