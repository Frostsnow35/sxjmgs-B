"""仅用于已确认演练的模拟器 HTTP 协议客户端。

该模块不判断模拟器 UI 当前是否为演练模式；``rehearsal_confirmed`` 是调用者
在用户确认演练状态后必须显式传入的本地护栏。
"""

from __future__ import annotations

import json
import threading
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import requests

Point = tuple[float, float]


class SimulatorProtocolError(RuntimeError):
    """模拟器动作无法被可信地视为已经执行时抛出。"""


class RehearsalClient:
    """串行执行并审计问题三演练接口请求。

    网络层仅在请求未取得 HTTP 响应时重试，最多三次。一个动作的请求
    载荷（包括 ``request_id``）在这三次尝试中保持不变，避免重复动作被
    误认为新动作。HTTP 状态或业务 ``accepted`` 字段不可信时立即停止。
    """

    _MAX_NETWORK_ATTEMPTS = 3

    def __init__(
        self,
        base_url: str,
        robot_id: str,
        log_path: Path,
        rehearsal_confirmed: bool = False,
        session: requests.Session | Any | None = None,
        timeout_s: float = 10.0,
    ) -> None:
        """初始化客户端，但不会在构造时发送 HTTP 请求。"""

        self._base_url = base_url.rstrip("/")
        self._robot_id = robot_id
        self._log_path = Path(log_path)
        self._rehearsal_confirmed = rehearsal_confirmed
        self._session = session if session is not None else requests.Session()
        self._timeout_s = timeout_s
        self._request_counter = 0
        self._serial_lock = threading.Lock()
        # 这些状态仅由已接受响应更新，供上层编排器计算下一步位置与频道。
        # 它们不是模拟器真值的替代品，审计仍以响应 JSONL 为准。
        self.pos: Point = (0.0, 0.0)
        self.current_channel = 1
        self.virtual_time_s = 0.0

    def enter(self) -> dict[str, Any]:
        """进入已获确认的演练。"""

        response = self._perform("enter", "/enter")
        self.pos = (0.0, 0.0)
        self.current_channel = 1
        self.virtual_time_s = _accepted_virtual_time(response, self.virtual_time_s)
        return response

    def measure(self, position: Point, channel: int) -> dict[str, Any]:
        """在 ``position`` 对指定频道执行一次测向。"""

        response = self._perform(
            "measure",
            "/measure",
            position={"x": position[0], "y": position[1]},
            channel=channel,
        )
        self.pos = (float(position[0]), float(position[1]))
        self.current_channel = int(channel)
        self.virtual_time_s = _accepted_virtual_time(response, self.virtual_time_s)
        return response

    def clear(self, position: Point, channel: int) -> dict[str, Any]:
        """在 ``position`` 对指定频道尝试清除。"""

        response = self._perform(
            "clear",
            "/clear",
            position={"x": position[0], "y": position[1]},
            channel=channel,
        )
        self.pos = (float(position[0]), float(position[1]))
        self.virtual_time_s = _accepted_virtual_time(response, self.virtual_time_s)
        return response

    def exit(self) -> dict[str, Any]:
        """退出已获确认的演练。"""

        response = self._perform("exit", "/exit")
        self.virtual_time_s = _accepted_virtual_time(response, self.virtual_time_s)
        return response

    def _perform(self, action: str, path: str, **extra_fields: Any) -> dict[str, Any]:
        """在全局串行锁内构造一次动作并发送请求。"""

        self._require_rehearsal_confirmation()
        with self._serial_lock:
            payload = self._base_payload(action)
            payload.update(extra_fields)
            return self._post(path, payload)

    def _base_payload(self, action: str) -> dict[str, Any]:
        """为一个新动作生成唯一的、可重试的载荷。"""

        self._request_counter += 1
        return {
            "arena_id": "default",
            "robot_id": self._robot_id,
            "request_id": f"{action}-{self._request_counter}",
        }

    def _require_rehearsal_confirmation(self) -> None:
        """阻止未显式确认演练的所有协议动作。"""

        if self._rehearsal_confirmed is not True:
            raise SimulatorProtocolError(
                "rehearsal confirmation is required before simulator requests"
            )

    def _post(self, path: str, payload: dict[str, Any]) -> dict[str, Any]:
        """发送同一载荷，网络失败时至多重试三次。"""

        for attempt in range(1, self._MAX_NETWORK_ATTEMPTS + 1):
            try:
                response = self._session.post(
                    f"{self._base_url}{path}",
                    json=payload,
                    headers={"Content-Type": "application/json"},
                    timeout=self._timeout_s,
                )
            except Exception as error:
                self._write_log(
                    path=path,
                    payload=payload,
                    attempt=attempt,
                    http_status=None,
                    response=None,
                    exception=error,
                )
                if attempt == self._MAX_NETWORK_ATTEMPTS:
                    raise SimulatorProtocolError(
                        f"network failure after {self._MAX_NETWORK_ATTEMPTS} attempts: {error}"
                    ) from error
                continue

            http_status = response.status_code
            if not 200 <= http_status < 300:
                try:
                    error_body = response.json()
                except Exception as error:
                    self._write_log(
                        path=path,
                        payload=payload,
                        attempt=attempt,
                        http_status=http_status,
                        response=None,
                        exception=error,
                    )
                else:
                    self._write_log(
                        path=path,
                        payload=payload,
                        attempt=attempt,
                        http_status=http_status,
                        response=error_body,
                        exception=None,
                    )
                raise SimulatorProtocolError(f"HTTP {http_status}")
            try:
                data = response.json()
            except Exception as error:
                self._write_log(
                    path=path,
                    payload=payload,
                    attempt=attempt,
                    http_status=http_status,
                    response=None,
                    exception=error,
                )
                raise SimulatorProtocolError(f"invalid JSON response for {path}: {error}") from error

            self._write_log(
                path=path,
                payload=payload,
                attempt=attempt,
                http_status=http_status,
                response=data,
                exception=None,
            )
            if not isinstance(data, dict) or data.get("accepted") is not True:
                raise SimulatorProtocolError(f"response for {path} does not set accepted to true")
            return data

        raise AssertionError("network retry loop must return or raise")

    def _write_log(
        self,
        *,
        path: str,
        payload: dict[str, Any],
        attempt: int,
        http_status: int | None,
        response: Any,
        exception: Exception | None,
    ) -> None:
        """将一次 HTTP 尝试追加为一行 UTF-8 JSONL。"""

        logged_payload = payload.copy()
        if "robot_id" in logged_payload:
            logged_payload["robot_id"] = "<redacted>"
        record = {
            "timestamp_utc": datetime.now(timezone.utc).isoformat(),
            "path": path,
            "attempt": attempt,
            "payload": logged_payload,
            "http_status": http_status,
            "response": response,
            "exception": None
            if exception is None
            else f"{type(exception).__name__}: {exception}",
        }
        self._log_path.parent.mkdir(parents=True, exist_ok=True)
        with self._log_path.open("a", encoding="utf-8", newline="\n") as log_file:
            log_file.write(json.dumps(record, ensure_ascii=False, default=str))
            log_file.write("\n")


def _accepted_virtual_time(response: dict[str, Any], fallback: float) -> float:
    """从已验证的成功响应读取有限虚拟时刻，缺失时保留上一个状态。"""

    value = response.get("virtual_time_s", fallback)
    if isinstance(value, bool):
        return fallback
    try:
        numeric_value = float(value)
    except (TypeError, ValueError, OverflowError):
        return fallback
    return numeric_value if numeric_value >= 0.0 else fallback
