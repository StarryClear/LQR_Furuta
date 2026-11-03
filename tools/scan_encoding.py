# -*- coding: utf-8 -*-
"""scan_encoding.py -- 扫描工程内文本文件的真实编码，找出乱码来源。

判定顺序：先试 UTF-8（严格），失败再试 GB18030，都失败则标 UNKNOWN。
输出：文件路径、判定编码、是否带 BOM、含中文行数、样例。
"""
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
EXTS = {".v", ".sv", ".cst", ".sdc", ".tcl", ".gprj", ".user", ".md", ".txt",
        ".py", ".ps1", ".mem", ".log", ".json"}
SKIP_DIRS = {"impl"}


def has_cjk(s):
    return any("\u4e00" <= ch <= "\u9fff" for ch in s)


def sniff(path):
    b = open(path, "rb").read()
    if b.startswith(b"\xef\xbb\xbf"):
        return "UTF-8-BOM", b[3:].decode("utf-8", "replace"), True
    try:
        return "UTF-8", b.decode("utf-8"), False
    except UnicodeDecodeError:
        pass
    try:
        return "GB18030", b.decode("gb18030"), False
    except UnicodeDecodeError:
        return "UNKNOWN", b.decode("latin-1"), False


def main():
    rows = []
    for dirpath, dirnames, filenames in os.walk(ROOT):
        dirnames[:] = [d for d in dirnames if d not in SKIP_DIRS]
        for fn in filenames:
            if os.path.splitext(fn)[1].lower() not in EXTS:
                continue
            p = os.path.join(dirpath, fn)
            enc, text, bom = sniff(p)
            cjk = [ln for ln in text.splitlines() if has_cjk(ln)]
            rows.append((os.path.relpath(p, ROOT), enc, bom, len(cjk), (cjk[0][:60] if cjk else "")))

    rows.sort(key=lambda r: (r[1], r[0]))
    print("%-52s %-11s %-4s %5s  %s" % ("file", "encoding", "BOM", "cjk", "sample"))
    print("-" * 140)
    for r in rows:
        print("%-52s %-11s %-4s %5d  %s" % (r[0][:52], r[1], "yes" if r[2] else "", r[3], r[4]))
    print()
    from collections import Counter
    c = Counter(r[1] for r in rows)
    print("summary:", dict(c))


if __name__ == "__main__":
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    main()
