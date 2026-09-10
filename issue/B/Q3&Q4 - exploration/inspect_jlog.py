from pathlib import Path
import json, sys
p = Path(sys.argv[1])
b = p.read_bytes()
print("size", len(b))
print("head", b[:20])
start = b.find(b'{"package_type')
print("json start", start)
dec = json.JSONDecoder()
s = b[start:].decode("utf-8", errors="replace")
obj, end = dec.raw_decode(s)
print("json end rel", end, "abs", start + end)
print("envelope keys", list(obj.keys()))
print("created", obj.get("created_at_utc"), "case", obj.get("case_code"), "run", obj.get("practice_run_no"))
tail = b[start + end:]
print("bytes after envelope", len(tail), "head", tail[:64])
print("tail ascii head", tail[:200])
# try search for JSON in tail
i2 = tail.find(b'{')
print("first { in tail", i2)
if i2 >= 0:
    try:
        s2 = tail[i2:].decode("utf-8", errors="replace")
        o2, e2 = dec.raw_decode(s2)
        print("second json keys", list(o2.keys()))
        print("second json first 500", json.dumps(o2, ensure_ascii=False)[:500])
    except Exception as e:
        print("second json parse fail", e)
