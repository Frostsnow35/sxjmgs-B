"""问题三、四的仅演练安全入口。

该脚本不会判断模拟器 UI；操作者必须先人工确认当前界面是相应问题的
“演练测试”，再传入 ``--rehearsal-confirmed``。它不包含正式测试路径，
没有该标志时不会构造客户端、不会连接端口、更不会调用 ``/enter``。
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path
from time import monotonic
from typing import Any


SCRIPT_ROOT = Path(__file__).resolve().parent
PROJECT_ROOT = SCRIPT_ROOT.parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))
if str(SCRIPT_ROOT) not in sys.path:
    sys.path.insert(0, str(SCRIPT_ROOT))

import strategy
from src.problem_3.protocol import RehearsalClient, SimulatorProtocolError


REQUEST_TIMEOUT_S = 2.0
SAFETY_MARGIN_S = 17.0
DEFAULT_BASE_URL = "http://127.0.0.1:2026"


class RehearsalTimeLimit(RuntimeError):
    """在新业务动作前真实时间余量不足时中止编排。"""


class ActionView:
    """将协议 JSON 响应适配为探索策略所需的只读属性。"""

    def __init__(self, payload: dict[str, Any]) -> None:
        self.measure_result = payload.get("measure_result")
        self.svd_deg = payload.get("svd_deg")
        self.clear_result = payload.get("clear_result")


class GuardedStrategyClient:
    """为策略提供位置状态和真实时限门禁的演练客户端适配器。"""

    def __init__(self, client: RehearsalClient) -> None:
        self._client = client
        self.pos = (0.0, 0.0)
        self.current_channel = 1
        self.virtual_time = 0.0
        self._deadline_s: float | None = None
        self.entered = False

    def enter(self) -> dict[str, Any]:
        """进入已人工确认的演练，并从响应建立唯一的现实时间截止线。"""

        response = self._client.enter()
        # 一旦 /enter 已被接受，即使剩余时长字段异常，也必须在调用方 finally
        # 中尽力 /exit，不能把已进入的会话误标为“尚未进入”。
        self.entered = True
        self._sync_state()
        remaining = response.get("remaining_real_duration_s")
        if isinstance(remaining, bool):
            raise RehearsalTimeLimit("remaining_real_duration_s is invalid")
        try:
            remaining_s = float(remaining)
        except (TypeError, ValueError, OverflowError) as error:
            raise RehearsalTimeLimit("remaining_real_duration_s is missing or invalid") from error
        if not math.isfinite(remaining_s) or remaining_s <= SAFETY_MARGIN_S:
            raise RehearsalTimeLimit("remaining_real_duration_s has no safe action margin")
        self._deadline_s = monotonic() + remaining_s
        return response

    def _check_before_action(self) -> None:
        if self._deadline_s is None or monotonic() + SAFETY_MARGIN_S >= self._deadline_s:
            raise RehearsalTimeLimit("real-time safety margin reached")

    def _sync_state(self) -> None:
        self.pos = self._client.pos
        self.current_channel = self._client.current_channel
        self.virtual_time = self._client.virtual_time_s

    def measure(self, position: tuple[float, float], channel: int) -> ActionView:
        """在时间余量充足时执行一次测量。"""

        self._check_before_action()
        response = self._client.measure(position, channel)
        self._sync_state()
        return ActionView(response)

    def clear(self, position: tuple[float, float], channel: int) -> ActionView:
        """在时间余量充足时执行一次清除尝试。"""

        self._check_before_action()
        response = self._client.clear(position, channel)
        self._sync_state()
        return ActionView(response)

    def exit(self) -> dict[str, Any]:
        """退出已进入的演练；退出动作不受新业务动作门禁限制。"""

        response = self._client.exit()
        self._sync_state()
        return response


def build_parser() -> argparse.ArgumentParser:
    """构造无正式测试选项的命令行参数。"""

    parser = argparse.ArgumentParser(description="仅限已人工确认的 Q3/Q4 演练；不启动或判断正式测试")
    parser.add_argument("--mode", choices=("q3", "q4"), required=True)
    parser.add_argument("--robot-id", required=True)
    parser.add_argument("--rehearsal-confirmed", action="store_true")
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--log", required=True, type=Path)
    parser.add_argument("--base-url", default=DEFAULT_BASE_URL)
    return parser


def write_summary(path: Path, summary: dict[str, Any]) -> None:
    """原子性不足时宁可拒绝覆盖既有证据。"""

    path.parent.mkdir(parents=True, exist_ok=True)
    if path.exists():
        raise FileExistsError(f"refusing to overwrite existing summary: {path}")
    path.write_text(json.dumps(summary, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def main(argv: list[str] | None = None) -> int:
    """运行一次显式确认的演练，或者安全地拒绝。"""

    args = build_parser().parse_args(argv)
    if args.log.exists() or args.log.is_dir():
        raise FileExistsError(f"refusing to append to an existing audit log: {args.log}")
    if args.output.is_dir():
        raise IsADirectoryError(f"summary path is a directory: {args.output}")
    if not args.rehearsal_confirmed:
        write_summary(
            args.output,
            {
                "schema": "q3_q4_rehearsal_summary",
                "version": "1.0",
                "mode": args.mode,
                "status": "refused",
                "reason": "rehearsal_confirmation_required",
                "reported_total_sources": None,
            },
        )
        return 2

    raw_client = RehearsalClient(
        args.base_url,
        args.robot_id,
        args.log,
        rehearsal_confirmed=True,
        timeout_s=REQUEST_TIMEOUT_S,
    )
    client = GuardedStrategyClient(raw_client)
    summary: dict[str, Any] = {
        "schema": "q3_q4_rehearsal_summary",
        "version": "1.0",
        "mode": args.mode,
        "run_scope": "operator_confirmed_simulation_rehearsal_not_official",
        "reported_total_sources": None,
        "status": "incomplete",
    }
    try:
        client.enter()
        result = strategy.run_q3(client) if args.mode == "q3" else strategy.run_q4(client)
        summary.update(
            {
                "status": "completed" if result["failed"] == 0 else "incomplete",
                "cleared_channels": result["cleared"],
                "failed_channels": result["failed"],
                "detected_channels": len(result["active_channels"]),
                "scan_point_count": len(result["survey_points"]),
                "virtual_time_s": client.virtual_time,
            }
        )
    except (SimulatorProtocolError, RehearsalTimeLimit, RuntimeError, ValueError) as error:
        summary.update({"status": "stopped", "reason": f"{type(error).__name__}: {error}"})
    finally:
        if client.entered:
            try:
                client.exit()
            except SimulatorProtocolError as error:
                summary.setdefault("exit_error", f"{type(error).__name__}: {error}")
        summary["last_virtual_time_s"] = client.virtual_time
        write_summary(args.output, summary)
    return 0 if summary["status"] == "completed" else 1


if __name__ == "__main__":
    raise SystemExit(main())
