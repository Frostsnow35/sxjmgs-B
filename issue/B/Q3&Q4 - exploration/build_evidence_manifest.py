"""构建 Q3/Q4 演练的脱敏证据清单，不连接模拟器。

清单以客户端 `/exit` 的 UTC 时间戳与模拟器行为目录中的
``*.result.json`` 结束时刻配对，因而不会把脚本参数 ``--mode q4``
误当作模拟器实际进入的问题四。原始 JLOG 信封只在内存中读取必要
元数据；输出不会包含队号、服务票据、密钥、案例编码或加密载荷。
"""

from __future__ import annotations

import argparse
import hashlib
import json
import statistics
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


SCRIPT_ROOT = Path(__file__).resolve().parent
PROJECT_ROOT = SCRIPT_ROOT.parents[2]
DEFAULT_SIMULATOR_LOG_DIR = (
    PROJECT_ROOT / "emulator" / "Jammers-simulator" / "JammersSimulatorData" / "behavior-logs"
)
DEFAULT_JLOG_DIR = SCRIPT_ROOT.parent / "JLOG"
PAIR_TOLERANCE_SECONDS = 5.0


@dataclass(frozen=True)
class ResultSidecar:
    """模拟器侧、与一次演练结束相伴生成的非敏感结果摘要。"""

    path: Path
    problem_no: int
    case_code: str
    ended_at_utc: datetime
    jammer_count: int
    omnidirectional_count: int
    directional_count: int
    sha256: str


def sha256_file(path: Path) -> str:
    """返回文件内容哈希，供不公开原始材料时进行一致性核对。"""

    digest = hashlib.sha256()
    with path.open("rb") as file:
        for block in iter(lambda: file.read(1 << 20), b""):
            digest.update(block)
    return digest.hexdigest()


def parse_utc(value: str) -> datetime:
    """解析模拟器 ISO-8601 UTC 字段。"""

    parsed = datetime.fromisoformat(value.replace("Z", "+00:00"))
    if parsed.tzinfo is None:
        raise ValueError(f"timestamp has no timezone: {value}")
    return parsed.astimezone(timezone.utc)


def read_jsonl(path: Path) -> list[dict[str, Any]]:
    """读取一份客户端 JSONL；坏行直接作为证据错误而非静默忽略。"""

    rows: list[dict[str, Any]] = []
    for line_number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), start=1):
        if not line.strip():
            continue
        value = json.loads(line)
        if not isinstance(value, dict):
            raise ValueError(f"{path.name}:{line_number} is not a JSON object")
        rows.append(value)
    return rows


def read_result_sidecars(directory: Path) -> tuple[ResultSidecar, ...]:
    """读取所有可用于时间配对的模拟器结果侧文件。"""

    sidecars: list[ResultSidecar] = []
    for path in sorted(directory.glob("*.result.json")):
        raw = json.loads(path.read_text(encoding="utf-8"))
        try:
            sidecars.append(
                ResultSidecar(
                    path=path,
                    problem_no=int(raw["problem_no"]),
                    case_code=str(raw["case_code"]),
                    ended_at_utc=parse_utc(str(raw["ended_at_utc"])),
                    jammer_count=int(raw["jammer_count"]),
                    omnidirectional_count=int(raw["omnidirectional_jammer_count"]),
                    directional_count=int(raw["directional_jammer_count"]),
                    sha256=sha256_file(path),
                )
            )
        except (KeyError, TypeError, ValueError) as error:
            raise ValueError(f"invalid result sidecar: {path}") from error
    return tuple(sidecars)


def read_jlog_index(directory: Path) -> dict[str, dict[str, Any]]:
    """按案例编码索引 JLOG，但只保留脱敏的演练属性和内容哈希。"""

    index: dict[str, dict[str, Any]] = {}
    for path in sorted(directory.rglob("*.jlog")):
        binary = path.read_bytes()
        start = binary.find(b'{"package_type')
        if start < 0:
            continue
        try:
            envelope, _ = json.JSONDecoder().raw_decode(binary[start:].decode("utf-8", errors="replace"))
            case_code = str(envelope["case_code"])
            index[case_code] = {
                "problem_no": int(envelope.get("problem_no")),
                "is_practice": envelope.get("package_type") == "practice_behavior_log",
                "formal_index_is_null": envelope.get("formal_index") is None,
                "sha256": sha256_file(path),
            }
        except (KeyError, TypeError, ValueError, json.JSONDecodeError):
            continue
    return index


def accepted_response_rows(rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    """筛选已被模拟器接受的响应行。"""

    return [
        row
        for row in rows
        if row.get("type") == "response"
        and isinstance(row.get("response"), dict)
        and row["response"].get("accepted") is True
    ]


def client_summary(path: Path) -> dict[str, Any]:
    """抽取一份自留客户端日志的可配对、可统计字段。"""

    rows = read_jsonl(path)
    accepted = accepted_response_rows(rows)
    exits = [row for row in accepted if row.get("path") == "/exit"]
    if len(exits) != 1:
        raise ValueError(f"{path.name} must contain exactly one accepted /exit")
    exit_row = exits[0]
    exit_timestamp = float(exit_row["wall_clock"])
    exit_time = datetime.fromtimestamp(exit_timestamp, tz=timezone.utc)
    measures = [row for row in accepted if row.get("path") == "/measure"]
    clears = [row for row in accepted if row.get("path") == "/clear"]
    clear_success = sum(row["response"].get("clear_result") == "success" for row in clears)
    clear_miss = sum(row["response"].get("clear_result") == "no_target_in_range" for row in clears)
    last_virtual_time = float(accepted[-1]["response"].get("virtual_time_s", 0.0))
    return {
        "sha256": sha256_file(path),
        "exit_time_utc": exit_time.isoformat().replace("+00:00", "Z"),
        "accepted_enter": sum(row.get("path") == "/enter" for row in accepted),
        "accepted_measure": len(measures),
        "accepted_clear": len(clears),
        "clear_success": clear_success,
        "clear_no_target_in_range": clear_miss,
        "accepted_user_exit": sum(
            row.get("path") == "/exit" and row["response"].get("exit_reason") == "user_exit"
            for row in accepted
        ),
        "virtual_time_s": last_virtual_time,
        "_exit_datetime": exit_time,
    }


def nearest_sidecar(exit_time: datetime, sidecars: tuple[ResultSidecar, ...]) -> tuple[ResultSidecar | None, float | None]:
    """返回时差最小且在允许窗口内的结果侧文件。"""

    if not sidecars:
        return None, None
    candidate = min(sidecars, key=lambda item: abs((item.ended_at_utc - exit_time).total_seconds()))
    delta = (candidate.ended_at_utc - exit_time).total_seconds()
    return (candidate, delta) if abs(delta) <= PAIR_TOLERANCE_SECONDS else (None, None)


def classify(runner_mode: str, sidecar: ResultSidecar | None) -> str:
    """根据模拟器实际题号而非脚本文件名分类一次运行。"""

    if sidecar is None:
        return "unmatched"
    expected_problem = {"q3": 3, "q4": 4}[runner_mode]
    if sidecar.problem_no == expected_problem:
        return f"canonical_{runner_mode}"
    return "mode_mismatch"


def median(values: list[float]) -> float | None:
    """对空序列返回 None，避免用伪零掩盖缺失证据。"""

    return statistics.median(values) if values else None


def aggregate(rows: list[dict[str, Any]]) -> dict[str, Any]:
    """汇总已配对的模拟器侧源数和客户端动作记录。

    该函数建立的是两类本地证据间的一致性核验，不把客户端成功响应
    单独包装为不可质疑的外部独立证明。字段名称刻意保留其来源，供
    论文在引用时区分“模拟器侧计数”和“客户端侧响应”。
    """

    source_total = sum(row["simulator_result"]["jammer_count"] for row in rows)
    success_total = sum(row["client"]["clear_success"] for row in rows)
    virtual_total = sum(row["client"]["virtual_time_s"] for row in rows)
    per_round = [
        row["client"]["virtual_time_s"] / row["simulator_result"]["jammer_count"]
        for row in rows
        if row["simulator_result"]["jammer_count"] > 0
    ]
    return {
        "runs": len(rows),
        "simulator_sidecar_source_total": source_total,
        "client_clear_success_total": success_total,
        "paired_clear_success_to_source_ratio": success_total / source_total if source_total else None,
        "all_paired_runs_source_count_equals_client_success": all(
            row["simulator_result"]["jammer_count"] == row["client"]["clear_success"]
            for row in rows
        ),
        "accepted_measure_total": sum(row["client"]["accepted_measure"] for row in rows),
        "accepted_clear_total": sum(row["client"]["accepted_clear"] for row in rows),
        "clear_no_target_in_range_total": sum(
            row["client"]["clear_no_target_in_range"] for row in rows
        ),
        "virtual_time_total_s": virtual_total,
        "virtual_time_mean_s": virtual_total / len(rows) if rows else None,
        "aggregate_full_process_time_per_source_s": virtual_total / source_total if source_total else None,
        "round_full_process_time_per_source_mean_s": statistics.mean(per_round) if per_round else None,
        "round_full_process_time_per_source_median_s": median(per_round),
    }


def build_manifest(exploration_root: Path, simulator_log_dir: Path, jlog_dir: Path) -> dict[str, Any]:
    """从已有文件重建确定性的、不含身份字段的证据清单。"""

    sidecars = read_result_sidecars(simulator_log_dir)
    jlog_index = read_jlog_index(jlog_dir)
    rows: list[dict[str, Any]] = []
    linked_case_codes: set[str] = set()
    for runner_mode in ("q3", "q4"):
        for client_log in sorted(exploration_root.glob(f"robot_{runner_mode}_round*.jsonl")):
            summary = client_summary(client_log)
            sidecar, delta = nearest_sidecar(summary.pop("_exit_datetime"), sidecars)
            category = classify(runner_mode, sidecar)
            simulator_result: dict[str, Any] | None = None
            jlog: dict[str, Any] | None = None
            if sidecar is not None:
                linked_case_codes.add(sidecar.case_code)
                simulator_result = {
                    "problem_no": sidecar.problem_no,
                    "jammer_count": sidecar.jammer_count,
                    "omnidirectional_jammer_count": sidecar.omnidirectional_count,
                    "directional_jammer_count": sidecar.directional_count,
                    "sidecar_sha256": sidecar.sha256,
                    "paired_time_delta_s": delta,
                }
                jlog = jlog_index.get(sidecar.case_code)
            round_number = int(client_log.stem.rsplit("round", maxsplit=1)[1])
            rows.append(
                {
                    "runner_mode": runner_mode,
                    "round": round_number,
                    "classification": category,
                    "client": summary,
                    "simulator_result": simulator_result,
                    "jlog_in_archive": jlog,
                }
            )

    canonical_q3 = [row for row in rows if row["classification"] == "canonical_q3"]
    canonical_q4 = [row for row in rows if row["classification"] == "canonical_q4"]
    mode_mismatches = [row for row in rows if row["classification"] == "mode_mismatch"]
    archive_case_codes = set(jlog_index)
    return {
        "schema": "q3_q4_rehearsal_evidence_manifest",
        "version": "1.1",
        "scope": "official_simulator_practice_only_not_formal_test",
        "pairing_rule": {
            "key": "accepted_exit_wall_clock_to_result_ended_at_utc",
            "tolerance_s": PAIR_TOLERANCE_SECONDS,
            "identity_fields_excluded": ["team_no", "server_ticket", "keys", "case_code"],
        },
        "runs": rows,
        "aggregates": {
            "q3": aggregate(canonical_q3),
            "q4": aggregate(canonical_q4),
            "mode_mismatch_run_count": len(mode_mismatches),
            "archived_jlog_count": len(jlog_index),
            "canonical_q3_archived_jlog_count": sum(row["jlog_in_archive"] is not None for row in canonical_q3),
            "canonical_q4_archived_jlog_count": sum(row["jlog_in_archive"] is not None for row in canonical_q4),
            "mode_mismatch_archived_jlog_count": sum(
                row["jlog_in_archive"] is not None for row in mode_mismatches
            ),
            "archived_jlog_not_linked_to_a_client_run": len(archive_case_codes - linked_case_codes),
        },
        "interpretation": {
            "q3": "Only canonical_q3 rows may support the paired Q3 rehearsal consistency check: simulator-side source counts versus client-side accepted clear successes.",
            "q4": "Only canonical_q4 rows may support the paired Q4 rehearsal consistency check: simulator-side source counts versus client-side accepted clear successes.",
            "mode_mismatch": "A runner filename or requested mode never overrides the simulator-result problem number.",
            "evidence_boundary": "The manifest is a reproducible local pairing of simulator-side result summaries and client audit logs. It is not a formal-test certificate, and it does not make LocalEnv results or an unmatched JLOG part of the reported experiment.",
            "raw_jlog": "Original JLOG files remain local supporting material and are not safe for public Git because their envelopes contain identity-related metadata.",
        },
    }


def main(argv: list[str] | None = None) -> int:
    """提供显式路径参数，便于在不依赖当前工作目录的情况下复建清单。"""

    parser = argparse.ArgumentParser(description="生成 Q3/Q4 演练脱敏证据清单；不连接模拟器")
    parser.add_argument("--exploration-root", type=Path, default=SCRIPT_ROOT)
    parser.add_argument("--simulator-log-dir", type=Path, default=DEFAULT_SIMULATOR_LOG_DIR)
    parser.add_argument("--jlog-dir", type=Path, default=DEFAULT_JLOG_DIR)
    parser.add_argument("--output", type=Path, default=SCRIPT_ROOT / "evidence_manifest.json")
    args = parser.parse_args(argv)
    manifest = build_manifest(args.exploration_root, args.simulator_log_dir, args.jlog_dir)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
