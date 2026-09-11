# -*- coding: utf-8 -*-
"""Extract text from a PDF (per page) using pymupdf."""
import sys, io

def main():
    src = sys.argv[1]
    out = sys.argv[2]
    import fitz
    doc = fitz.open(src)
    parts = []
    for i, page in enumerate(doc):
        txt = page.get_text("text")
        parts.append(f"\n===== [PAGE {i+1}/{len(doc)}] =====\n" + txt)
    full = "".join(parts)
    with io.open(out, "w", encoding="utf-8") as f:
        f.write(full)
    # summary to stdout: total chars and first lines of first non-empty pages
    nchars = sum(len(p) for p in parts)
    print(f"PAGES={len(doc)} CHARS={nchars}")
    # print head of the whole text
    head = full[:4000]
    sys.stdout.reconfigure(encoding="utf-8")
    print(head)

if __name__ == "__main__":
    main()
