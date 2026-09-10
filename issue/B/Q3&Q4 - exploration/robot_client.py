"""Official-simulator HTTP robot client.

Before a drill test:
    * start the official emulator and log in;
    * start a Q3/Q4 drill test;
    * wait until the interface is open (the emulator shows the countdown has
      finished), then run:

          python robot_client.py --mode q3 --team <your_team_id> [--base http://127.0.0.1:2026]

The program records every request / response in robot_q3.jsonl or
robot_q4.jsonl in the current directory.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import time
import urllib.error
import urllib.request
from pathlib import Path

import strategy

BASE_URL = "http://127.0.0.1:2026"
HTTP_TIMEOUT = 5.0
REAL_MARGIN_S = 20.0


class OfficialClient:
    def __init__(self, base_url: str, team_id: str, mode: str, log_path=None):
        self.base_url = base_url.rstrip("/")
        self.team_id = team_id
        self.mode = mode
        self.request_counter = 0
        self.pos = (0.0, 0.0)
        self.current_channel = 1
        self.virtual_time = 0.0
        self.entered = False
        self.deadline = None
        self.log_path = Path(log_path) if log_path else Path(f"robot_{mode}.jsonl")
        self.log_file = self.log_path.open("w", encoding="utf-8")
        self._log_meta()

    # ------------------------------------------------------------------
    def _log_meta(self):
        self.log_file.write(json.dumps({
            "type": "meta", "mode": self.mode,
            "base_url": self.base_url,
            "started_unix": time.time(),
        }, ensure_ascii=False) + "\n")
        self.log_file.flush()

    def _next_request_id(self) -> str:
        self.request_counter += 1
        return f"{self.mode}-{int(time.time()*1000)}-{self.request_counter}"

    def post(self, path: str, payload: dict, retry_deadline=None):
        """POST one action with idempotent retry on network errors."""
        deadline = retry_deadline if retry_deadline is not None else self.deadline
        if deadline is not None and time.monotonic() > deadline:
            raise TimeoutError("real-time deadline reached")
        data = json.dumps(payload, separators=(",", ":")).encode("utf-8")
        req = urllib.request.Request(
            self.base_url + path,
            data=data,
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        while True:
            try:
                with urllib.request.urlopen(req, timeout=HTTP_TIMEOUT) as resp:
                    body = resp.read().decode("utf-8")
                    obj = json.loads(body)
                self.log_file.write(json.dumps({
                    "type": "response", "path": path, "request_id": payload["request_id"],
                    "http_status": 200, "response": obj,
                    "wall_clock": time.time(),
                }, ensure_ascii=False) + "\n")
                self.log_file.flush()
                return obj
            except urllib.error.HTTPError as e:
                # 400/409/415 etc. are real responses: do not blindly retry.
                try:
                    body = e.read().decode("utf-8")
                    obj = json.loads(body)
                except Exception:
                    obj = {"accepted": False, "http_error": e.code, "body": body}
                self.log_file.write(json.dumps({
                    "type": "response", "path": path, "request_id": payload["request_id"],
                    "http_status": e.code, "response": obj,
                    "wall_clock": time.time(),
                }, ensure_ascii=False) + "\n")
                self.log_file.flush()
                return obj
            except (urllib.error.URLError, TimeoutError, ConnectionError, OSError) as e:
                # Interface not open / network glitch: retry exactly the same
                # request with the same request_id.
                if deadline is not None and time.monotonic() > deadline:
                    raise TimeoutError("retry deadline reached") from e
                self.log_file.write(json.dumps({
                    "type": "retry", "path": path, "request_id": payload["request_id"],
                    "error": str(e), "wall_clock": time.time(),
                }, ensure_ascii=False) + "\n")
                self.log_file.flush()
                time.sleep(0.5)

    # ------------------------------------------------------------------
    def enter(self, wait: bool = True, max_wait_s: float = 3600.0):
        payload = {
            "arena_id": "default",
            "robot_id": self.team_id,
            "request_id": self._next_request_id(),
        }
        retry_until = time.monotonic() + max_wait_s
        while True:
            obj = self.post("/enter", payload, retry_deadline=retry_until)
            if obj.get("accepted") is True:
                self.entered = True
                self.virtual_time = float(obj.get("virtual_time_s", 0.0))
                remaining = float(obj.get("remaining_real_duration_s", 1200.0))
                self.deadline = time.monotonic() + max(1.0, remaining - REAL_MARGIN_S)
                return obj
            if not wait:
                return obj
            time.sleep(0.5)

    def exit(self):
        payload = {
            "arena_id": "default",
            "robot_id": self.team_id,
            "request_id": self._next_request_id(),
        }
        return self.post("/exit", payload)

    def _action_payload(self, request_id: str, pos, channel):
        return {
            "arena_id": "default",
            "robot_id": self.team_id,
            "request_id": request_id,
            "position": {"x": float(pos[0]), "y": float(pos[1])},
            "channel": int(channel),
        }

    def measure(self, pos, channel):
        payload = self._action_payload(self._next_request_id(), pos, channel)
        obj = self.post("/measure", payload)
        if obj.get("accepted") is not True:
            raise RuntimeError(f"/measure not accepted: {obj}")
        self.virtual_time = float(obj.get("virtual_time_s", self.virtual_time))
        self.pos = (float(pos[0]), float(pos[1]))
        self.current_channel = int(channel)
        return _ActionView(obj)

    def clear(self, pos, channel):
        payload = self._action_payload(self._next_request_id(), pos, channel)
        obj = self.post("/clear", payload)
        if obj.get("accepted") is not True:
            raise RuntimeError(f"/clear not accepted: {obj}")
        self.virtual_time = float(obj.get("virtual_time_s", self.virtual_time))
        self.pos = (float(pos[0]), float(pos[1]))
        # /clear does not change current_channel
        return _ActionView(obj)

    def close(self):
        try:
            self.log_file.close()
        except Exception:
            pass


class _ActionView:
    def __init__(self, obj: dict):
        self.obj = obj
        self.accepted = obj.get("accepted") is True
        self.measure_result = obj.get("measure_result")
        self.svd_deg = obj.get("svd_deg")
        self.clear_result = obj.get("clear_result")
        self.virtual_time_s = obj.get("virtual_time_s", 0.0)


def wait_until_interface_open(base_url: str, timeout_s: float = 3600.0):
    """Optional helper: wait for the emulator's HTTP port to answer /enter."""
    # We cannot send /enter repeatedly (it would enter when open), so this is
    # only used for a TCP-level port probe before the actual enter call.
    import socket
    from urllib.parse import urlparse
    host = urlparse(base_url).hostname or "127.0.0.1"
    port = urlparse(base_url).port or 2026
    start = time.monotonic()
    while time.monotonic() - start < timeout_s:
        try:
            with socket.create_connection((host, port), timeout=1.0):
                return True
        except OSError:
            time.sleep(0.5)
    return False


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mode", choices=["q3", "q4"], required=True)
    ap.add_argument("--team", required=True, help="contest team id")
    ap.add_argument("--base", default=BASE_URL)
    ap.add_argument("--no-wait", action="store_true")
    ap.add_argument("--enter-wait", type=float, default=3600.0,
                    help="maximum seconds to keep retrying /enter")
    args = ap.parse_args()

    client = OfficialClient(args.base, args.team, args.mode)
    try:
        if not args.no_wait:
            wait_until_interface_open(args.base, timeout_s=3600.0)
        enter = client.enter(wait=True, max_wait_s=args.enter_wait)
        if enter.get("accepted") is not True:
            print("enter failed:", enter, file=sys.stderr)
            return 1
        print(f"entered, remaining_real={enter.get('remaining_real_duration_s')}s")

        if args.mode == "q3":
            result = strategy.run_q3(client)
        else:
            result = strategy.run_q4(client)

        client.exit()
        print(json.dumps({
            "mode": args.mode,
            "cleared": result["cleared"],
            "failed": result["failed"],
            "active_channels": result["active_channels"],
            "virtual_time_s": client.virtual_time,
        }, ensure_ascii=False))
        print("log:", client.log_path)
        return 0
    except Exception as e:
        print("ERROR:", repr(e), file=sys.stderr)
        try:
            if client.entered:
                client.exit()
        except Exception:
            pass
        return 2
    finally:
        client.close()


if __name__ == "__main__":
    raise SystemExit(main())
