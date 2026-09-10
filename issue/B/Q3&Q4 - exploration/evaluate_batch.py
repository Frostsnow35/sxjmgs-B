"""Summarize the 10+10 practice rounds and cross-check the exported JLOG files."""
from __future__ import annotations

import json
import statistics
from pathlib import Path

BASE = Path(__file__).resolve().parent
JLOG = BASE.parent / "JLOG"


def load_jsonl(path: Path):
    out = []
    if not path.exists():
        return out
    with path.open("r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if line:
                out.append(json.loads(line))
    return out


def summarize_mode(mode: str):
    rows = load_jsonl(BASE / f"drill_batch_{mode}.jsonl")
    cleared = [r["cleared"] for r in rows]
    avg = [r["avg_time"] for r in rows if r.get("avg_time") is not None]
    virt = [r["virtual_time"] for r in rows]
    fails = sum(r.get("failed", 0) for r in rows)
    clear_succ = sum(r.get("clear_success", 0) for r in rows)
    clear_req = sum(r.get("clear_actions", 0) for r in rows)
    measure = sum(r.get("measure_actions", 0) for r in rows)
    print(f"\n===== {mode.upper()}  {len(rows)} rounds =====")
    print("cleared per round:", cleared)
    print(f"total cleared: {sum(cleared)}  mean: {statistics.mean(cleared):.1f}")
    print(f"failed rounds/total failed sources: {fails}")
    print(f"avg_time per cleared source: mean={statistics.mean(avg):.2f}s "
          f"median={statistics.median(avg):.2f}s min={min(avg):.2f}s max={max(avg):.2f}s")
    print(f"virtual_time total: mean={statistics.mean(virt):.2f}s "
          f"min={min(virt):.2f}s max={max(virt):.2f}s")
    print(f"measures={measure} clear_requests={clear_req} clear_success={clear_succ}")
    return {
        "mode": mode,
        "rounds": len(rows),
        "cleared_per_round": cleared,
        "total_cleared": sum(cleared),
        "failed_sources": fails,
        "avg_time_mean": statistics.mean(avg) if avg else None,
        "avg_time_median": statistics.median(avg) if avg else None,
        "avg_time_min": min(avg) if avg else None,
        "avg_time_max": max(avg) if avg else None,
        "virtual_time_mean": statistics.mean(virt) if virt else None,
        "virtual_time_min": min(virt) if virt else None,
        "virtual_time_max": max(virt) if virt else None,
        "measure_actions": measure,
        "clear_requests": clear_req,
        "clear_success": clear_succ,
    }


def jlog_codes(folder: str):
    folder = JLOG / folder
    rows = []
    if folder.exists():
        for p in sorted(folder.glob("*.jlog")):
            b = p.read_bytes()
            start = b.find(b'{"package_type')
            if start < 0:
                start = b.find(b'{')
            if start < 0:
                continue
            dec = json.JSONDecoder()
            try:
                obj, _ = dec.raw_decode(b[start:].decode("utf-8", errors="replace"))
                rows.append({
                    "case_code": obj.get("case_code"),
                    "problem_no": obj.get("problem_no"),
                    "created_at_utc": obj.get("created_at_utc"),
                    "file": p.name,
                })
            except Exception:
                rows.append({"file": p.name, "parse_error": True})
    return rows


def main():
    q3 = summarize_mode("q3")
    q4 = summarize_mode("q4")
    j3 = jlog_codes("p3-tenTrial-1")
    j4 = jlog_codes("p4-tenTrial-1")
    print("\nJLOG p3 count:", len(j3))
    print("JLOG p4 count:", len(j4))
    if len(j4) < q4["rounds"]:
        print(f"NOTE: Q4 rounds={q4['rounds']} but only {len(j4)} JLOG files found; "
              "one Q4 practice log may not have been exported yet.")
    out = {
        "q3": q3,
        "q4": q4,
        "jlog_p3_count": len(j3),
        "jlog_p4_count": len(j4),
        "jlog_p3_codes": j3,
        "jlog_p4_codes": j4,
    }
    out_path = BASE / "drill_batch_evaluation.json"
    out_path.write_text(json.dumps(out, indent=2, ensure_ascii=False), encoding="utf-8")
    print("\nwrote", out_path)


if __name__ == "__main__":
    main()
