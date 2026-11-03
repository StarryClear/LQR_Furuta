# -*- coding: utf-8 -*-
"""check_links.py -- 检查所有 Markdown 里的相对链接是否指向真实存在的文件。

用法：python tools/check_links.py
"""
import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SKIP = {"impl", ".git"}
LINK = re.compile(r"\]\(([^)#]+?)(?:#[^)]*)?\)")


def main():
    bad = 0
    total = 0
    for dp, dn, fn in os.walk(ROOT):
        dn[:] = [d for d in dn if d not in SKIP]
        for f in fn:
            if not f.lower().endswith(".md"):
                continue
            p = os.path.join(dp, f)
            txt = open(p, encoding="utf-8").read()
            for m in LINK.finditer(txt):
                tgt = m.group(1).strip()
                if tgt.startswith(("http://", "https://", "mailto:")):
                    continue
                total += 1
                tp = os.path.normpath(os.path.join(dp, tgt))
                if not os.path.exists(tp):
                    print("  MISSING  %-38s -> %s" % (os.path.relpath(p, ROOT), tgt))
                    bad += 1
    print("检查相对链接 %d 个，失效 %d 个" % (total, bad))
    return 1 if bad else 0


if __name__ == "__main__":
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    sys.exit(main())
