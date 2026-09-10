"""离线验证问题三演练编排入口。"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Callable
from uuid import uuid4

import pytest

from scripts import run_problem_3_rehearsal as runner
from src.localization_geometry import LocalizationResult
from src.problem_3.strategy import pending_channels_after_scan


def test_single_direction_channel_remains_pending_after_scan() -> None:
    """单条有效示向度不能因观测数不足而从待处理集合中消失。"""

    observations = {7: [((0.0, 0.0), 20.0)]}

    assert pending_channels_after_scan(observations, cleared_channels=set()) == (7,)


def _artifact_paths() -> tuple[Path, Path]:
    """在工作树可写的 tmp 目录分配不依赖系统 tmp ACL 的测试产物路径。"""

    root = Path(__file__).resolve().parents[1] / "tmp"
    token = uuid4().hex
    return root / f"problem-3-summary-{token}.json", root / f"problem-3-actions-{token}.jsonl"


def _accepted(**fields: Any) -> dict[str, Any]:
    """构造协议已接受的假响应。"""

    return {"accepted": True, **fields}


class FakeClient:
    """显式注入的离线客户端，记录编排层的动作顺序。"""

    def __init__(
        self,
        *,
        enter_response: dict[str, Any] | None = None,
        measure_handler: Callable[[tuple[float, float], int], dict[str, Any]] | None = None,
        clear_handler: Callable[[tuple[float, float], int], dict[str, Any]] | None = None,
        exit_response: dict[str, Any] | None = None,
    ) -> None:
        self.actions: list[tuple[Any, ...]] = []
        self.enter_response = enter_response or _accepted(
            remaining_real_duration_s=100.0, virtual_time_s=1.0
        )
        self.measure_handler = measure_handler or (
            lambda _position, _channel: _accepted(
                measure_result="no_signal", virtual_time_s=2.0
            )
        )
        self.clear_handler = clear_handler or (
            lambda _position, _channel: _accepted(
                clear_result="no_target_in_range", virtual_time_s=3.0
            )
        )
        self.exit_response = exit_response or _accepted(virtual_time_s=99.0)

    def enter(self) -> dict[str, Any]:
        self.actions.append(("enter",))
        return self.enter_response

    def measure(self, position: tuple[float, float], channel: int) -> dict[str, Any]:
        self.actions.append(("measure", position, channel))
        return self.measure_handler(position, channel)

    def clear(self, position: tuple[float, float], channel: int) -> dict[str, Any]:
        self.actions.append(("clear", position, channel))
        return self.clear_handler(position, channel)

    def exit(self) -> dict[str, Any]:
        self.actions.append(("exit",))
        return self.exit_response


def test_main_refuses_unconfirmed_rehearsal_before_constructing_client() -> None:
    """没有显式确认时，入口不能构建真实客户端或执行任意动作。"""

    output_path, log_path = _artifact_paths()
    factory_calls: list[tuple[Any, ...]] = []

    def factory(*args: Any, **kwargs: Any) -> FakeClient:
        factory_calls.append((*args, kwargs))
        return FakeClient()

    result = runner.main(
        ["--robot-id", "private-team-id", "--output", str(output_path), "--log", str(log_path)],
        client_factory=factory,
    )

    assert result == 2
    assert factory_calls == []
    summary = json.loads(output_path.read_text(encoding="utf-8"))
    assert summary["status"] == "refused"
    assert "private-team-id" not in output_path.read_text(encoding="utf-8")


def test_help_does_not_construct_client_or_issue_network_actions() -> None:
    """CLI 帮助仅解析参数，不能触及客户端。"""

    factory_calls: list[tuple[Any, ...]] = []

    def factory(*args: Any, **kwargs: Any) -> FakeClient:
        factory_calls.append((*args, kwargs))
        return FakeClient()

    with pytest.raises(SystemExit, match="0"):
        runner.main(["--help"], client_factory=factory)

    assert factory_calls == []


def test_runner_scans_all_points_remeasures_and_uses_closed_region_localization(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """七点全频道扫描后，单条示向度必须复测、闭角域定位并分离摘要状态。"""

    output_path, log_path = _artifact_paths()
    first_point = (0.0, 0.0)
    second_point = runner.guaranteed_second_point(first_point, 10.0)

    def measure(position: tuple[float, float], channel: int) -> dict[str, Any]:
        if position == first_point and channel == 7:
            return _accepted(measure_result="direction", svd_deg=10.0, virtual_time_s=10.0)
        if position == second_point and channel == 7:
            return _accepted(measure_result="direction", svd_deg=80.0, virtual_time_s=11.0)
        return _accepted(measure_result="no_signal", virtual_time_s=12.0)

    client = FakeClient(
        measure_handler=measure,
        clear_handler=lambda position, channel: _accepted(
            clear_result="success"
            if (position, channel) == ((10.0, 20.0), 7)
            else "no_target_in_range",
            virtual_time_s=13.0,
        ),
    )
    locate_calls: list[tuple[Any, ...]] = []

    def locate(observations: Any, *, error_deg: float, target_radius_m: float) -> LocalizationResult:
        locate_calls.append((observations, error_deg, target_radius_m))
        return LocalizationResult(
            "point", ((10.0, 20.0),), 0.0, (((10.0, 20.0), (10.0, 20.0)),), True, ((10.0, 20.0),)
        )

    monkeypatch.setattr(runner, "locate_from_bearings", locate)

    summary = runner.run_rehearsal(
        client=client,
        output_path=output_path,
        action_log_path=log_path,
        monotonic=lambda: 0.0,
        command_summary={"base_url": "omitted"},
    )

    scan_actions = client.actions[1:141]
    assert len(scan_actions) == 140
    assert [action[2] for action in scan_actions] == list(range(1, 21)) * 7
    assert [action[1] for action in scan_actions[::20]] == list(runner.initial_scan_points())
    assert client.actions[141] == ("measure", second_point, 7)
    assert client.actions[142] == ("clear", (10.0, 20.0), 7)
    assert client.actions[143] == ("exit",)
    assert locate_calls == [
        (((0.0, 0.0, 10.0), (*second_point, 80.0)), 1.0, 1800.0)
    ]
    assert summary["cleared_channels"] == [7]
    assert summary["unresolved_channels"] == []
    assert summary["discovered_channels"] == [7]
    assert summary["reported_total_sources"] is None
    assert summary["scan_point_count"] == 7
    assert summary["observation_counts"] == {"7": 2}
    assert summary["localization_statuses"] == {"7": "point"}
    assert summary["virtual_time_s"] == 99.0
    persisted = json.loads(output_path.read_text(encoding="utf-8"))
    assert persisted == summary


def test_no_signal_second_measure_and_failed_clear_keep_channels_unresolved(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """复测无信号及 near 清除失败均不能让已经发现的频道静默消失。"""

    output_path, log_path = _artifact_paths()

    def measure(position: tuple[float, float], channel: int) -> dict[str, Any]:
        if position == (0.0, 0.0) and channel == 3:
            return _accepted(measure_result="direction", svd_deg=0.0, virtual_time_s=10.0)
        if position == (0.0, 0.0) and channel == 4:
            return _accepted(measure_result="near", virtual_time_s=11.0)
        return _accepted(measure_result="no_signal", virtual_time_s=12.0)

    client = FakeClient(measure_handler=measure)
    monkeypatch.setattr(
        runner,
        "locate_from_bearings",
        lambda *_args, **_kwargs: LocalizationResult("empty", (), None, (), None, ()),
    )
    monkeypatch.setattr(runner, "clearance_grid", lambda _bounds, *, radius_m: ((1.0, 1.0),))

    summary = runner.run_rehearsal(
        client=client,
        output_path=output_path,
        action_log_path=log_path,
        monotonic=lambda: 0.0,
        command_summary={},
    )

    assert client.actions[5] == ("clear", (0.0, 0.0), 4)
    assert ("measure", (750.0, 600.0), 3) in client.actions
    assert summary["discovered_channels"] == [3, 4]
    assert summary["cleared_channels"] == []
    assert summary["unresolved_channels"] == [3, 4]
    assert summary["observation_counts"] == {"3": 1}


@pytest.mark.parametrize(
    ("enter_response", "reason_fragment"),
    [
        (_accepted(virtual_time_s=1.0), "missing_or_invalid"),
        (_accepted(remaining_real_duration_s=17.0, virtual_time_s=1.0), "safety_margin"),
    ],
)
def test_enter_remaining_duration_controls_deadline_without_assuming_1200_seconds(
    enter_response: dict[str, Any], reason_fragment: str
) -> None:
    """缺失或小于 17 秒安全余量的 enter 响应必须阻止后续 measure。"""

    output_path, log_path = _artifact_paths()

    def forbidden_measure(_position: tuple[float, float], _channel: int) -> dict[str, Any]:
        raise AssertionError("deadline stop 后不得测量")

    client = FakeClient(enter_response=enter_response, measure_handler=forbidden_measure)

    summary = runner.run_rehearsal(
        client=client,
        output_path=output_path,
        action_log_path=log_path,
        monotonic=lambda: 0.0,
        command_summary={},
    )

    assert summary["status"] == "stopped"
    assert reason_fragment in summary["stop_reason"]
    assert [action[0] for action in client.actions] == ["enter", "exit"]


@pytest.mark.parametrize("invalid_argument", ["--output", "--log"])
def test_main_preflights_directory_artifact_path_before_constructing_client(
    invalid_argument: str,
) -> None:
    """目录不能作为摘要或日志文件时，必须在构造客户端前拒绝。"""

    output_path, log_path = _artifact_paths()
    invalid_path = output_path.parent / f"problem-3-directory-{uuid4().hex}"
    invalid_path.mkdir(parents=True)
    existing_summary = "keep-existing-summary"
    output_path.write_text(existing_summary, encoding="utf-8")
    factory_calls: list[tuple[Any, ...]] = []

    def factory(*args: Any, **kwargs: Any) -> FakeClient:
        factory_calls.append((*args, kwargs))
        return FakeClient()

    result = runner.main(
        [
            "--robot-id",
            "private-team-id",
            "--rehearsal-confirmed",
            "--output",
            str(invalid_path if invalid_argument == "--output" else output_path),
            "--log",
            str(invalid_path if invalid_argument == "--log" else log_path),
        ],
        client_factory=factory,
    )

    assert result != 0
    assert factory_calls == []
    if invalid_argument == "--log":
        assert output_path.read_text(encoding="utf-8") == existing_summary


def test_main_uses_bounded_timeout_and_17_second_safety_margin() -> None:
    """真实客户端构造须限制单请求时长，17 秒余量时不能开始 measure。"""

    output_path, log_path = _artifact_paths()
    captured_kwargs: dict[str, Any] = {}

    def forbidden_measure(_position: tuple[float, float], _channel: int) -> dict[str, Any]:
        raise AssertionError("17 秒余量不足时不得测量")

    client = FakeClient(
        enter_response=_accepted(remaining_real_duration_s=17.0, virtual_time_s=1.0),
        measure_handler=forbidden_measure,
    )

    def factory(*_args: Any, **kwargs: Any) -> FakeClient:
        captured_kwargs.update(kwargs)
        return client

    result = runner.main(
        [
            "--robot-id",
            "private-team-id",
            "--rehearsal-confirmed",
            "--output",
            str(output_path),
            "--log",
            str(log_path),
        ],
        client_factory=factory,
    )

    assert result == 1
    assert runner.SAFETY_MARGIN_S == 17.0
    assert captured_kwargs == {"rehearsal_confirmed": True, "timeout_s": 2.0}
    assert [action[0] for action in client.actions] == ["enter", "exit"]
    summary = json.loads(output_path.read_text(encoding="utf-8"))
    assert summary["time_limit_policy"] == {
        "request_timeout_s": 2.0,
        "max_network_attempts_per_action": 3,
        "safety_margin_s": 17.0,
        "safety_margin_basis": "3*2s business retries + 3*2s exit retries + 5s reserve",
    }


def test_main_returns_nonzero_when_summary_write_fails(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """摘要持久化失败不能被吞掉并错误报告成功退出码。"""

    output_path, log_path = _artifact_paths()
    client = FakeClient()
    factory_calls: list[tuple[Any, ...]] = []

    def factory(*args: Any, **kwargs: Any) -> FakeClient:
        factory_calls.append((*args, kwargs))
        return client

    def fail_write(_path: Path, _summary: Mapping[str, Any]) -> None:
        raise OSError("summary write denied")

    monkeypatch.setattr(runner, "_write_summary", fail_write)

    result = runner.main(
        [
            "--robot-id",
            "private-team-id",
            "--rehearsal-confirmed",
            "--output",
            str(output_path),
            "--log",
            str(log_path),
        ],
        client_factory=factory,
    )

    assert result != 0
    assert len(factory_calls) == 1
    assert client.actions[0] == ("enter",)
    assert client.actions[-1] == ("exit",)


@pytest.mark.parametrize(
    ("status", "vertices", "expected_bounds"),
    [
        ("empty", (), (-1800.0, -1800.0, 1800.0, 1800.0)),
        ("unbounded", ((1.0, 2.0),), (-1800.0, -1800.0, 1800.0, 1800.0)),
        ("disk_clipped", ((1.0, 2.0),), (-1800.0, -1800.0, 1800.0, 1800.0)),
        ("point", (), (-1800.0, -1800.0, 1800.0, 1800.0)),
        ("polygon", ((-3.0, 5.0), (7.0, 11.0)), (-3.0, 5.0, 7.0, 11.0)),
    ],
)
def test_clearance_fallback_uses_full_disk_only_for_unbounded_status_or_no_vertices(
    monkeypatch: pytest.MonkeyPatch,
    status: str,
    vertices: tuple[tuple[float, float], ...],
    expected_bounds: tuple[float, float, float, float],
) -> None:
    """圆弧/无界/空集或无顶点时不能以不充分顶点缩小回退区域。"""

    output_path, log_path = _artifact_paths()
    bounds_calls: list[tuple[float, float, float, float]] = []

    def measure(position: tuple[float, float], channel: int) -> dict[str, Any]:
        if position == (0.0, 0.0) and channel == 1:
            return _accepted(measure_result="direction", svd_deg=0.0, virtual_time_s=10.0)
        return _accepted(measure_result="no_signal", virtual_time_s=11.0)

    monkeypatch.setattr(
        runner,
        "locate_from_bearings",
        lambda *_args, **_kwargs: LocalizationResult(status, vertices, None, (), None, ()),
    )
    monkeypatch.setattr(
        runner,
        "clearance_grid",
        lambda bounds, *, radius_m: bounds_calls.append(bounds) or ((1.0, 1.0),),
    )
    client = FakeClient(
        measure_handler=measure,
        clear_handler=lambda _position, _channel: _accepted(clear_result="success", virtual_time_s=12.0),
    )

    summary = runner.run_rehearsal(
        client=client,
        output_path=output_path,
        action_log_path=log_path,
        monotonic=lambda: 0.0,
        command_summary={},
    )

    assert bounds_calls == [expected_bounds]
    assert summary["cleared_channels"] == [1]


def test_invalid_measure_response_stops_and_exits_without_continuing() -> None:
    """不合规格响应要留下失败摘要并停止，而不是假定测量成功。"""

    output_path, log_path = _artifact_paths()
    client = FakeClient(
        measure_handler=lambda _position, _channel: _accepted(virtual_time_s=2.0)
    )

    summary = runner.run_rehearsal(
        client=client,
        output_path=output_path,
        action_log_path=log_path,
        monotonic=lambda: 0.0,
        command_summary={},
    )

    assert summary["status"] == "failed"
    assert "response_schema_error" in summary["stop_reason"]
    assert [action[0] for action in client.actions] == ["enter", "measure", "exit"]


def test_unknown_clear_result_stops_after_near_and_exits_without_new_actions() -> None:
    """非协议枚举的 clear_result 不能被当作普通清除失败后继续扫描。"""

    output_path, log_path = _artifact_paths()

    def measure(position: tuple[float, float], channel: int) -> dict[str, Any]:
        if position == (0.0, 0.0) and channel == 1:
            return _accepted(measure_result="near", virtual_time_s=2.0)
        raise AssertionError("未知 clear_result 后不得开始新的 measure")

    client = FakeClient(
        measure_handler=measure,
        clear_handler=lambda _position, _channel: _accepted(
            clear_result="unexpected", virtual_time_s=3.0
        ),
    )

    summary = runner.run_rehearsal(
        client=client,
        output_path=output_path,
        action_log_path=log_path,
        monotonic=lambda: 0.0,
        command_summary={},
    )

    assert summary["status"] == "failed"
    assert "response_schema_error" in summary["stop_reason"]
    assert [action[0] for action in client.actions] == ["enter", "measure", "clear", "exit"]
