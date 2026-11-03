# -*- coding: utf-8 -*-
"""fix_fences.py -- 修复 ANTI_DISTURBANCE_LOG.md 里被破坏的代码围栏。

用状态机扫一遍：维护"当前是否在代码块内"。
  在块内、整行只有 1 个反引号  -> 这是收尾围栏，补成 ```
  不在块内、整行是 1 个反引号 + 语言名（可能夹 TAB/VT）-> 补成 ```语言名
  已经是 ``` 的行原样保留并翻转状态
只改这几行，其余字节（含 CRLF）原样保留。
"""
import io
import os
import re
import sys

P = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                 "tools", "experiments_20261102", "ANTI_DISTURBANCE_LOG.md")

LANG_ALIAS = {"ext": "text", "erilog": "verilog", "extblock": "text"}


def main():
    raw = io.open(P, "r", encoding="utf-8", newline="").read()
    lines = raw.split("\n")
    in_code = False
    fixed = 0

    for i, ln in enumerate(lines):
        cr = "\r" if ln.endswith("\r") else ""
        core = ln.rstrip("\r")
        st = core.strip()

        if st.startswith("```"):
            in_code = not in_code
            continue

        if not st.startswith("`"):
            continue

        # 去掉开头反引号和夹在中间的控制字符，取语言名
        body = st[1:]
        body = re.sub(r"^[\t\x0b\x0c\r ]+", "", body)
        lang = re.sub(r"[^A-Za-z]", "", body)

        if in_code:
            if lang == "":
                lines[i] = "```" + cr
                print("  line %d: %-18r -> '```'（收尾）" % (i + 1, st))
                fixed += 1
                in_code = False
            continue

        if lang in LANG_ALIAS or lang in ("text", "verilog", "bash", "python"):
            new = "```" + LANG_ALIAS.get(lang, lang)
            lines[i] = new + cr
            print("  line %d: %-18r -> %r（开头）" % (i + 1, st, new))
            fixed += 1
            in_code = True

    io.open(P, "w", encoding="utf-8", newline="").write("\n".join(lines))
    print("fixed =", fixed)

    txt = io.open(P, encoding="utf-8").read()
    n = sum(1 for x in txt.split("\n") if x.strip().startswith("```"))
    print("``` 总数 = %d %s" % (n, "（闭合正常）" if n % 2 == 0 else "⚠ 奇数，需人工看"))


if __name__ == "__main__":
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    main()
