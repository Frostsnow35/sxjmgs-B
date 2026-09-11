# -*- coding: utf-8 -*-
"""Download a URL to a file using urllib (with UA + TLS fallback)."""
import sys, ssl, urllib.request

def main():
    url = sys.argv[1]
    out = sys.argv[2]
    ctx = ssl.create_default_context()
    req = urllib.request.Request(url, headers={"User-Agent": "Mozilla/5.0 (research)"})
    with urllib.request.urlopen(req, context=ctx, timeout=60) as r:
        data = r.read()
    with open(out, "wb") as f:
        f.write(data)
    print(f"OK {len(data)} bytes -> {out}")

if __name__ == "__main__":
    main()
