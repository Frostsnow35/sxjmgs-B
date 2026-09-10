"""离线检验问题三演练协议的防误触与可追溯行为。"""

from __future__ import annotations

import json
import threading
import time
import uuid
from collections import deque
from pathlib import Path
from typing import Any

import pytest

from src.problem_3.protocol import RehearsalClient, SimulatorProtocolError


class FakeResponse:
    """仅模拟协议客户端实际读取的 HTTP 响应接口。"""

    def __init__(self, status_code: int, body: Any) -> None:
        self.status_code = status_code
        self._body = body
        self.json_calls = 0

    def json(self) -> dict[str, Any]:
        self.json_calls += 1
        if isinstance(self._body, BaseException):
            raise self._body
        return self._body


class FakeSession:
    """按给定结果序列返回响应，且保留真实发送内容供行为断言。"""

    def __init__(self, outcomes: list[FakeResponse | BaseException]) -> None:
        self._outcomes = deque(outcomes)
        self.calls: list[dict[str, Any]] = []

    def post(
        self,
        url: str,
        *,
        json: dict[str, Any],
        headers: dict[str, str],
        timeout: float,
    ) -> FakeResponse:
        self.calls.append(
            {"url": url, "json": json.copy(), "headers": headers.copy(), "timeout": timeout}
        )
        outcome = self._outcomes.popleft()
        if isinstance(outcome, BaseException):
            raise outcome
        return outcome


class OverlapDetectingSession:
    """在 post 内让出执行权，以检查客户端是否确实将请求串行化。"""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self.active_requests = 0
        self.maximum_active_requests = 0

    def post(
        self,
        url: str,
        *,
        json: dict[str, Any],
        headers: dict[str, str],
        timeout: float,
    ) -> FakeResponse:
        with self._lock:
            self.active_requests += 1
            self.maximum_active_requests = max(self.maximum_active_requests, self.active_requests)
        time.sleep(0.03)
        with self._lock:
            self.active_requests -= 1
        return FakeResponse(200, {"accepted": True})


def new_log_path() -> Path:
    """返回工作树内的独立日志名，避开本机无权限的系统临时目录。"""

    return Path(__file__).resolve().parents[1] / "tmp" / f"protocol-{uuid.uuid4().hex}.jsonl"


def make_client(log_path: Path, session: Any, *, rehearsal_confirmed: bool = True) -> RehearsalClient:
    """建立仅使用假会话的客户端，杜绝离线测试访问模拟器。"""

    return RehearsalClient(
        "http://127.0.0.1:2026",
        "team",
        log_path,
        rehearsal_confirmed=rehearsal_confirmed,
        session=session,
    )


@pytest.mark.parametrize(
    ("action", "arguments"),
    [
        ("enter", ()),
        ("measure", ((10.0, -2.0), 3)),
        ("clear", ((10.0, -2.0), 3)),
        ("exit", ()),
    ],
)
def test_client_refuses_every_action_without_rehearsal_confirmation(
    action: str, arguments: tuple[Any, ...]
) -> None:
    session = FakeSession([])
    client = make_client(new_log_path(), session, rehearsal_confirmed=False)

    with pytest.raises(SimulatorProtocolError, match="rehearsal"):
        getattr(client, action)(*arguments)

    assert session.calls == []


@pytest.mark.parametrize("confirmation", ["false", 1, [], object()])
@pytest.mark.parametrize(
    ("action", "arguments"),
    [
        ("enter", ()),
        ("measure", ((10.0, -2.0), 3)),
        ("clear", ((10.0, -2.0), 3)),
        ("exit", ()),
    ],
)
def test_client_requires_identity_true_for_every_action(
    confirmation: Any, action: str, arguments: tuple[Any, ...]
) -> None:
    session = FakeSession([])
    client = make_client(new_log_path(), session, rehearsal_confirmed=confirmation)

    with pytest.raises(SimulatorProtocolError, match="rehearsal"):
        getattr(client, action)(*arguments)

    assert session.calls == []


@pytest.mark.parametrize(
    ("action", "arguments"),
    [
        ("enter", ()),
        ("measure", ((10.0, -2.0), 3)),
        ("clear", ((10.0, -2.0), 3)),
        ("exit", ()),
    ],
)
def test_client_defaults_to_refusing_actions_without_confirmation(
    action: str, arguments: tuple[Any, ...]
) -> None:
    session = FakeSession([])
    client = RehearsalClient(
        "http://127.0.0.1:2026",
        "team",
        new_log_path(),
        session=session,
    )

    with pytest.raises(SimulatorProtocolError, match="rehearsal"):
        getattr(client, action)(*arguments)

    assert session.calls == []


def test_client_rejects_http_error_even_when_body_claims_accepted() -> None:
    log_path = new_log_path()
    response = FakeResponse(500, {"accepted": True})
    session = FakeSession([response])
    client = make_client(log_path, session)

    with pytest.raises(SimulatorProtocolError, match=r"^HTTP 500$"):
        client.enter()

    record = json.loads(log_path.read_text(encoding="utf-8"))
    assert record["path"] == "/enter"
    assert record["http_status"] == 500
    assert record["payload"]["robot_id"] == "<redacted>"
    assert record["response"] == {"accepted": True}
    assert record["exception"] is None
    assert response.json_calls == 1


def test_client_rejects_http_error_and_persists_invalid_body_parse_failure() -> None:
    log_path = new_log_path()
    response = FakeResponse(500, ValueError("must not parse HTTP error body"))
    session = FakeSession([response])
    client = make_client(log_path, session)

    with pytest.raises(SimulatorProtocolError, match=r"^HTTP 500$"):
        client.enter()

    record = json.loads(log_path.read_text(encoding="utf-8"))
    assert response.json_calls == 1
    assert record["http_status"] == 500
    assert record["payload"]["request_id"] == "enter-1"
    assert record["response"] is None
    assert record["exception"] == "ValueError: must not parse HTTP error body"


@pytest.mark.parametrize("accepted", [False, None, "true", 1])
def test_client_requires_boolean_true_accepted_field(accepted: Any) -> None:
    session = FakeSession([FakeResponse(200, {"accepted": accepted})])
    client = make_client(new_log_path(), session)

    with pytest.raises(SimulatorProtocolError, match="accepted"):
        client.enter()


def test_network_retry_reuses_the_exact_payload_and_request_id() -> None:
    log_path = new_log_path()
    session = FakeSession(
        [ConnectionError("transient disconnect"), FakeResponse(200, {"accepted": True})]
    )
    client = make_client(log_path, session)

    assert client.measure((12.5, -3.0), 4) == {"accepted": True}

    assert len(session.calls) == 2
    assert session.calls[0]["json"] == session.calls[1]["json"]
    assert session.calls[0]["json"]["request_id"].startswith("measure-")

    records = [
        json.loads(line)
        for line in log_path.read_text(encoding="utf-8").splitlines()
    ]
    assert records[0]["exception"] == "ConnectionError: transient disconnect"
    assert records[0]["http_status"] is None
    assert records[1]["http_status"] == 200
    assert records[1]["response"] == {"accepted": True}


def test_client_redacts_robot_id_in_jsonl_without_changing_http_payload() -> None:
    """审计日志不能泄露队号，但实际协议请求仍必须保留其身份字段。"""

    log_path = new_log_path()
    session = FakeSession([FakeResponse(200, {"accepted": True})])
    client = make_client(log_path, session)

    client.enter()

    log_text = log_path.read_text(encoding="utf-8")
    record = json.loads(log_text)
    assert session.calls[0]["json"]["robot_id"] == "team"
    assert record["payload"]["robot_id"] == "<redacted>"
    assert "team" not in log_text


def test_each_new_action_gets_a_fresh_request_id() -> None:
    session = FakeSession(
        [FakeResponse(200, {"accepted": True}), FakeResponse(200, {"accepted": True})]
    )
    client = make_client(new_log_path(), session)

    client.enter()
    client.measure((0.0, 0.0), 1)

    first_id = session.calls[0]["json"]["request_id"]
    second_id = session.calls[1]["json"]["request_id"]
    assert first_id != second_id
    assert first_id.startswith("enter-")
    assert second_id.startswith("measure-")


def test_client_stops_after_at_most_three_network_attempts() -> None:
    session = FakeSession([ConnectionError("lost")] * 3)
    client = make_client(new_log_path(), session)

    with pytest.raises(SimulatorProtocolError, match="network failure"):
        client.exit()

    assert len(session.calls) == 3
    assert {call["json"]["request_id"] for call in session.calls} == {"exit-1"}


def test_client_rejects_non_json_response_and_persists_exception() -> None:
    log_path = new_log_path()
    session = FakeSession([FakeResponse(200, ValueError("not JSON"))])
    client = make_client(log_path, session)

    with pytest.raises(SimulatorProtocolError, match="JSON"):
        client.enter()

    record = json.loads(log_path.read_text(encoding="utf-8"))
    assert record["http_status"] == 200
    assert record["exception"] == "ValueError: not JSON"


def test_client_serializes_calls_from_multiple_threads() -> None:
    session = OverlapDetectingSession()
    client = make_client(new_log_path(), session)
    failures: list[BaseException] = []

    def measure(channel: int) -> None:
        try:
            client.measure((0.0, 0.0), channel)
        except BaseException as error:  # pragma: no cover - asserted below
            failures.append(error)

    threads = [threading.Thread(target=measure, args=(channel,)) for channel in (1, 2)]
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join()

    assert failures == []
    assert session.maximum_active_requests == 1
