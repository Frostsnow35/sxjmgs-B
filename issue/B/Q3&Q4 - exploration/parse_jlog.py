"""Read .jlog practice behavior logs produced by the official emulator.

The file starts with a short binary header, then a JSON envelope, then a
binary/encrypted payload.  For practice logs the envelope itself contains the
important result fields, so this script extracts those fields for all logs in
the two JLOG sub-folders.
"""
from __future__ import annotations

import json
import sys
from pathlib import Path


def read_envelope(path: Path) -> dict | None:
    b = path.read_bytes()
    start = b.find(b'{"package_type')
    if start < 0:
        # Some files may have a different header; try the first '{'.
        start = b.find(b'{')
    if start < 0:
        return None
    s = b[start:].decode("utf-8", errors="replace")
    dec = json.JSONDecoder()
    try:
        obj, _ = dec.raw_decode(s)
        return obj
    except Exception:
        # Fall back: cut at the first occurrence of the start of a second
        # top-level JSON object (payload marker).
        return None


def summarize(obj: dict) -> dict:
    # Keep only safe/interesting fields; avoid dumping server tickets etc.
    keys = [
        "problem_no", "formal_index", "practice_run_no", "case_code",
        "created_at_utc",
    ]
    out = {k: obj.get(k) for k in keys}
    # The emulator v1.1.0 envelope usually carries summary fields as well.
    for k in obj.keys():
        if "summary" in k.lower() or "result" in k.lower() or "total" in k.lower():
            out[k] = obj[k]
    return out


def main():
    base = Path("JLOG")
    if len(sys.argv) > 1:
        base = Path(sys.argv[1])
    for folder in sorted(base.glob("p*-tenTrial-*")):
        logs = sorted(folder.glob("*.jlog"))
        print(f"\n===== {folder.name}  ({len(logs)} logs) =====")
        for path in logs:
            obj = read_envelope(path)
            if obj is None:
                print(path.name, "PARSE_FAIL")
                continue
            print(json.dumps(summarize(obj), ensure_ascii=False))


if __name__ == "__main__":
    main()
