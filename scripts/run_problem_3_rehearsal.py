"""问题三仅限已确认模拟演练的可审计编排入口。

本入口不判断模拟器 UI 模式；调用人必须在 UI 中确认后显式传入
``--rehearsal-confirmed``。未确认时不会构造 HTTP 客户端。
"""

from __future__ import annotations

import argparse
import hashlib
import json
import logging
import math
import sys
from collections.abc import Callable, Mapping, Sequence
from numbers import Real
from pathlib import Path
from time import monotonic as system_monotonic
from typing import Any, Protocol


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from src.localization_geometry import LocalizationResult, locate_from_bearings
from src.problem_3.protocol import RehearsalClient, SimulatorProtocolError
from src.problem_3.strategy import (
    Point,
    clearance_grid,
    guaranteed_second_point,
    initial_scan_points,
    pending_channels_after_scan,
)


LOGGER = logging.getLogger(__name__)
CHANNELS: tuple[int, ...] = tuple(range(1, 21))
TARGET_RADIUS_M = 1800.0
BEARING_ERROR_DEG = 1.0
CLEAR_RADIUS_M = 20.0
SAFETY_MARGIN_S = 5.0
SCHEMA_NAME = "problem_3_rehearsal_summary"
SCHEMA_VERSION = "1.0"
DEFAULT_BASE_URL = "http://127.0.0.1:2026"
FULL_TARGET_BOUNDS: tuple[float, float, float, float] = (
    -TARGET_RADIUS_M,
    -TARGET_RADIUS_M,
    TARGET_RADIUS_M,
    TARGET_RADIUS_M,
)


class RehearsalActions(Protocol):
    """编排器所需的最小客户端接口，便于离线注入假客户端。"""

    def enter(self) -> Mapping[str, Any]:
        """进入模拟演练。"""

    def measure(self, position: Point, channel: int) -> Mapping[str, Any]:
        """在指定位置测量频道。"""

    def clear(self, position: Point, channel: int) -> Mapping[str, Any]:
        """在指定位置尝试清除频道。"""

    def exit(self) -> Mapping[str, Any]:
        """退出模拟演练。"""


class ResponseSchemaError(RuntimeError):
    """协议客户端以外的响应格式不满足编排器所需字段。"""


def build_parser() -> argparse.ArgumentParser:
    """创建仅用于问题三模拟演练的严格命令行参数解析器。"""

    parser = argparse.ArgumentParser(
        description="仅问题三模拟演练；不判断 UI 模式。需在 UI 确认后显式授权。"
    )
    parser.add_argument("--robot-id", required=True, help="演练机器人标识（不会写入摘要）")
    parser.add_argument(
        "--rehearsal-confirmed",
        action="store_true",
        help="已由操作者确认当前为问题三模拟演练窗口",
    )
    parser.add_argument("--output", required=True, type=Path, help="UTF-8 汇总 JSON 输出路径")
    parser.add_argument("--log", required=True, type=Path, help="协议 JSONL 审计日志路径")
    parser.add_argument(
        "--base-url",
        default=DEFAULT_BASE_URL,
        help="已确认演练模拟器的基础地址（默认 2026 端口）",
    )
    return parser


def run_rehearsal(
    *,
    client: RehearsalActions,
    output_path: Path,
    action_log_path: Path,
    monotonic: Callable[[], float] = system_monotonic,
    command_summary: Mapping[str, Any],
) -> dict[str, Any]:
    """执行一次已获确认的离线可注入问题三演练流程。

    Args:
        client: 已由调用方显式确认后创建的协议客户端，测试中可传入 FakeClient。
        output_path: 脱敏汇总 JSON 的写入位置。
        action_log_path: ``RehearsalClient`` 负责写入的 JSONL 路径，仅在摘要中引用。
        monotonic: 可注入的单调时钟，用于由 ``/enter`` 响应建立真实时限。
        command_summary: 已脱敏的命令参数摘要；任何身份或凭据键均会被剔除。

    Returns:
        总是尽力写入并返回的结构化汇总。协议或格式错误会标记为失败且不继续动作。
    """

    observations: dict[int, list[tuple[Point, float]]] = {}
    discovered_channels: set[int] = set()
    cleared_channels: set[int] = set()
    localization_statuses: dict[int, str] = {}
    last_virtual_time_s: float | None = None
    remaining_real_duration_s: float | None = None
    deadline_s: float | None = None
    entered = False
    scan_point_count = 0
    status = "incomplete"
    stop_reason = "not_started"

    def accept_response(response: Mapping[str, Any], action: str) -> dict[str, Any]:
        """校验已接受响应并更新最后一个可追溯虚拟时间。"""

        nonlocal last_virtual_time_s
        if not isinstance(response, Mapping):
            raise ResponseSchemaError(f"{action} response must be an object")
        if response.get("accepted") is not True:
            raise ResponseSchemaError(f"{action} response must set accepted to true")
        normalized = dict(response)
        if "virtual_time_s" in normalized:
            last_virtual_time_s = _finite_real(
                normalized["virtual_time_s"], f"{action}.virtual_time_s"
            )
        return normalized

    def may_start_new_action() -> bool:
        """在每个非清理动作前保留五秒真实时间安全余量。"""

        nonlocal status, stop_reason
        if deadline_s is None:
            status = "stopped"
            stop_reason = "remaining_real_duration_s_missing_or_invalid"
            return False
        now_s = _finite_real(monotonic(), "monotonic")
        if now_s + SAFETY_MARGIN_S >= deadline_s:
            status = "stopped"
            stop_reason = "real_time_safety_margin_reached"
            return False
        return True

    def clear_at(position: Point, channel: int) -> bool:
        """在仍有时限余量时清除；只有 success 才改变清除状态。"""

        if not may_start_new_action():
            return False
        clear_response = accept_response(client.clear(position, channel), "clear")
        clear_result = clear_response.get("clear_result")
        if not isinstance(clear_result, str):
            raise ResponseSchemaError("clear response must include string clear_result")
        if clear_result == "success":
            cleared_channels.add(channel)
            return True
        return False

    def record_measurement(
        position: Point, channel: int, response: Mapping[str, Any]
    ) -> str:
        """校验一次测量并记录 direction 或 near 发现状态。"""

        measurement = accept_response(response, "measure")
        result = measurement.get("measure_result")
        if result not in {"direction", "near", "no_signal"}:
            raise ResponseSchemaError(
                "measure response must include measure_result of direction, near, or no_signal"
            )
        if result == "direction":
            bearing_deg = _finite_real(measurement.get("svd_deg"), "measure.svd_deg")
            discovered_channels.add(channel)
            observations.setdefault(channel, []).append((position, bearing_deg))
        elif result == "near":
            discovered_channels.add(channel)
        return result

    try:
        enter_response = accept_response(client.enter(), "enter")
        entered = True
        remaining_value = enter_response.get("remaining_real_duration_s")
        try:
            remaining_real_duration_s = _positive_or_zero_finite(
                remaining_value, "enter.remaining_real_duration_s"
            )
        except ValueError:
            status = "stopped"
            stop_reason = "remaining_real_duration_s_missing_or_invalid"
        else:
            deadline_s = _finite_real(monotonic(), "monotonic") + remaining_real_duration_s

        continue_execution = deadline_s is not None
        scan_points = initial_scan_points()
        if continue_execution:
            for scan_point in scan_points:
                point_started = False
                for channel in CHANNELS:
                    if not may_start_new_action():
                        continue_execution = False
                        break
                    point_started = True
                    scan_result = record_measurement(
                        scan_point, channel, client.measure(scan_point, channel)
                    )
                    if scan_result == "near":
                        clear_at(scan_point, channel)
                    if status == "stopped":
                        continue_execution = False
                        break
                if point_started:
                    scan_point_count += 1
                if not continue_execution:
                    break

        if continue_execution:
            for channel in pending_channels_after_scan(observations, cleared_channels):
                if len(observations[channel]) != 1:
                    continue
                if not may_start_new_action():
                    continue_execution = False
                    break
                first_position, first_bearing_deg = observations[channel][0]
                second_position = guaranteed_second_point(first_position, first_bearing_deg)
                second_result = record_measurement(
                    second_position,
                    channel,
                    client.measure(second_position, channel),
                )
                if second_result == "near":
                    clear_at(second_position, channel)
                if status == "stopped":
                    continue_execution = False
                    break

        if continue_execution:
            for channel in pending_channels_after_scan(observations, cleared_channels):
                if not may_start_new_action():
                    continue_execution = False
                    break
                channel_observations = tuple(
                    (position[0], position[1], bearing_deg)
                    for position, bearing_deg in observations[channel]
                )
                localization = locate_from_bearings(
                    channel_observations,
                    error_deg=BEARING_ERROR_DEG,
                    target_radius_m=TARGET_RADIUS_M,
                )
                localization_statuses[channel] = localization.status
                for center in localization.cover_centers:
                    clear_at(center, channel)
                    if channel in cleared_channels or status == "stopped":
                        break
                if channel in cleared_channels or status == "stopped":
                    if status == "stopped":
                        continue_execution = False
                        break
                    continue
                fallback_bounds = _fallback_bounds(localization)
                for candidate in clearance_grid(fallback_bounds, radius_m=CLEAR_RADIUS_M):
                    clear_at(candidate, channel)
                    if channel in cleared_channels or status == "stopped":
                        break
                if status == "stopped":
                    continue_execution = False
                    break

        if status not in {"stopped", "failed"}:
            unresolved_after_actions = discovered_channels - cleared_channels
            if unresolved_after_actions:
                status = "incomplete"
                stop_reason = "unresolved_channels_after_candidates"
            else:
                status = "completed"
                stop_reason = "completed"
    except SimulatorProtocolError as error:
        LOGGER.error("simulator protocol error; stopping rehearsal", exc_info=True)
        status = "failed"
        stop_reason = f"protocol_error: {type(error).__name__}: {error}"
    except ResponseSchemaError as error:
        LOGGER.error("invalid simulator response; stopping rehearsal", exc_info=True)
        status = "failed"
        stop_reason = f"response_schema_error: {type(error).__name__}: {error}"
    except Exception as error:  # pragma: no cover - defensive structured failure path
        LOGGER.exception("unexpected runner error; stopping rehearsal")
        status = "failed"
        stop_reason = f"runner_error: {type(error).__name__}: {error}"
    finally:
        if entered:
            try:
                # exit 是清理动作：即便安全余量已耗尽，也仅尽力执行一次，不再开启新工作。
                accept_response(client.exit(), "exit")
            except Exception as exit_error:
                LOGGER.error("unable to exit rehearsal cleanly", exc_info=True)
                exit_reason = f"exit_error: {type(exit_error).__name__}: {exit_error}"
                if status == "completed":
                    status = "failed"
                    stop_reason = exit_reason
                else:
                    stop_reason = f"{stop_reason}; {exit_reason}"

        summary = _build_summary(
            status=status,
            stop_reason=stop_reason,
            observations=observations,
            discovered_channels=discovered_channels,
            cleared_channels=cleared_channels,
            localization_statuses=localization_statuses,
            last_virtual_time_s=last_virtual_time_s,
            remaining_real_duration_s=remaining_real_duration_s,
            action_log_path=action_log_path,
            scan_point_count=scan_point_count,
            command_summary=command_summary,
        )
        try:
            _write_summary(output_path, summary)
        except Exception:  # pragma: no cover - filesystem error is logged for operator recovery
            LOGGER.exception("failed to persist rehearsal summary")
        return summary


def _fallback_bounds(localization: LocalizationResult) -> tuple[float, float, float, float]:
    """为定位结果选取保守清除框，避免圆弧或无界区域被顶点低估。"""

    if localization.status not in {"point", "segment", "polygon"} or not localization.vertices:
        return FULL_TARGET_BOUNDS
    xs, ys = zip(*localization.vertices)
    return (min(xs), min(ys), max(xs), max(ys))


def _finite_real(value: Any, name: str) -> float:
    """将响应/时钟中的非布尔有限实数转换为 float。"""

    if isinstance(value, bool) or not isinstance(value, Real):
        raise ValueError(f"{name} must be a finite real number")
    numeric_value = float(value)
    if not math.isfinite(numeric_value):
        raise ValueError(f"{name} must be a finite real number")
    return numeric_value


def _positive_or_zero_finite(value: Any, name: str) -> float:
    """校验 enter 的真实剩余秒数为非负有限数。"""

    numeric_value = _finite_real(value, name)
    if numeric_value < 0.0:
        raise ValueError(f"{name} must be non-negative")
    return numeric_value


def _fallback_command_summary(command_summary: Mapping[str, Any]) -> dict[str, Any]:
    """仅保留可公开复核的命令信息，排除身份和凭据。"""

    allowed_keys = {"rehearsal_confirmed", "output_filename", "log_filename"}
    return {
        key: command_summary[key]
        for key in sorted(allowed_keys)
        if key in command_summary
    }


def _build_summary(
    *,
    status: str,
    stop_reason: str,
    observations: Mapping[int, Sequence[tuple[Point, float]]],
    discovered_channels: set[int],
    cleared_channels: set[int],
    localization_statuses: Mapping[int, str],
    last_virtual_time_s: float | None,
    remaining_real_duration_s: float | None,
    action_log_path: Path,
    scan_point_count: int,
    command_summary: Mapping[str, Any],
) -> dict[str, Any]:
    """构造不含机器人标识、接口密钥或 UI 测试码的可审计摘要。"""

    safe_command_summary = _fallback_command_summary(command_summary)
    configuration = {
        "run_mode": "rehearsal",
        "channels": [CHANNELS[0], CHANNELS[-1]],
        "scan_points": len(initial_scan_points()),
        "target_radius_m": TARGET_RADIUS_M,
        "bearing_error_deg": BEARING_ERROR_DEG,
        "clear_radius_m": CLEAR_RADIUS_M,
        "real_time_safety_margin_s": SAFETY_MARGIN_S,
        "command_parameters": safe_command_summary,
    }
    configuration_hash = hashlib.sha256(
        json.dumps(configuration, ensure_ascii=False, sort_keys=True, separators=(",", ":")).encode("utf-8")
    ).hexdigest()
    return {
        "schema": SCHEMA_NAME,
        "version": SCHEMA_VERSION,
        "run_mode": "rehearsal",
        "status": status,
        "stop_reason": stop_reason,
        "cleared_channels": sorted(cleared_channels),
        "unresolved_channels": sorted(discovered_channels - cleared_channels),
        "discovered_channels": sorted(discovered_channels),
        "reported_total_sources": None,
        "virtual_time_s": last_virtual_time_s,
        "remaining_real_duration_s": remaining_real_duration_s,
        "action_log_path": str(action_log_path),
        "scan_point_count": scan_point_count,
        "configured_scan_point_count": len(initial_scan_points()),
        "observation_counts": {
            str(channel): len(channel_observations)
            for channel, channel_observations in sorted(observations.items())
        },
        "localization_statuses": {
            str(channel): localization_statuses[channel]
            for channel in sorted(localization_statuses)
        },
        "command_parameters": safe_command_summary,
        "configuration_hash": configuration_hash,
    }


def _write_summary(output_path: Path, summary: Mapping[str, Any]) -> None:
    """以 UTF-8 创建父目录并持久化汇总 JSON。"""

    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(
        json.dumps(summary, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )


def _refusal_summary(output_path: Path, action_log_path: Path) -> dict[str, Any]:
    """生成未确认演练时的零动作脱敏摘要。"""

    return _build_summary(
        status="refused",
        stop_reason="rehearsal_confirmation_required",
        observations={},
        discovered_channels=set(),
        cleared_channels=set(),
        localization_statuses={},
        last_virtual_time_s=None,
        remaining_real_duration_s=None,
        action_log_path=action_log_path,
        scan_point_count=0,
        command_summary={"rehearsal_confirmed": False, "output_filename": output_path.name, "log_filename": action_log_path.name},
    )


def main(
    argv: Sequence[str] | None = None,
    *,
    client_factory: Callable[..., RehearsalActions] = RehearsalClient,
) -> int:
    """解析参数并在显式确认后才构造协议客户端和执行演练。"""

    args = build_parser().parse_args(argv)
    if args.rehearsal_confirmed is not True:
        summary = _refusal_summary(args.output, args.log)
        try:
            _write_summary(args.output, summary)
        except Exception:
            LOGGER.exception("failed to persist refusal summary")
        LOGGER.error("refusing to create a rehearsal client without --rehearsal-confirmed")
        return 2

    client = client_factory(
        args.base_url,
        args.robot_id,
        args.log,
        rehearsal_confirmed=True,
    )
    summary = run_rehearsal(
        client=client,
        output_path=args.output,
        action_log_path=args.log,
        command_summary={
            "rehearsal_confirmed": True,
            "output_filename": args.output.name,
            "log_filename": args.log.name,
        },
    )
    return 0 if summary["status"] == "completed" else 1


if __name__ == "__main__":
    raise SystemExit(main())
