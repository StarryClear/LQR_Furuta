# -*- coding: utf-8 -*-
"""fix_encoding.py -- 把工程内所有文本文件统一成 UTF-8（无 BOM）。

处理的四种情况：
  1. UTF-16 LE/BE（带 BOM）  -> UTF-8   （Windows PowerShell 重定向的默认产物）
  2. UTF-8 with BOM          -> UTF-8   （PowerShell Set-Content -Encoding UTF8 的产物）
  3. GB18030（中文 Windows 的 ANSI，Gowin 控制台日志的原始编码）-> UTF-8
  4. UTF-8 但内容已是 mojibake（双重编码）-> 报告出来，不自动改（需人工确认）

用法：
    python tools/fix_encoding.py           # 只报告
    python tools/fix_encoding.py --apply   # 实际改写
"""
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
EXTS = {".v", ".sv", ".cst", ".sdc", ".tcl", ".gprj", ".user", ".md", ".txt",
        ".py", ".ps1", ".log", ".json", ".xml", ".cfg", ".do"}
SKIP_DIRS = {"impl", ".git"}


def detect(b):
    if b.startswith(b"\xff\xfe") and not b.startswith(b"\xff\xfe\x00\x00"):
        return "UTF-16LE-BOM", b[2:].decode("utf-16-le")
    if b.startswith(b"\xfe\xff"):
        return "UTF-16BE-BOM", b[2:].decode("utf-16-be")
    if b.startswith(b"\xef\xbb\xbf"):
        return "UTF-8-BOM", b[3:].decode("utf-8")
    try:
        return "UTF-8", b.decode("utf-8")
    except UnicodeDecodeError:
        pass
    try:
        return "GB18030", b.decode("gb18030")
    except UnicodeDecodeError:
        return "UNKNOWN", None


def is_mojibake(text):
    """启发式：出现 '鐗' '锛' '浣' 这类"UTF-8 被当 GBK 读"的高频字，判为乱码。"""
    marks = "鐗锛浣佸鏄銆锟鏂寮闂閿鍚娴"
    hit = sum(text.count(c) for c in marks)
    return hit >= 4


def main():
    apply = "--apply" in sys.argv
    changed, bad = [], []
    for dirpath, dirnames, filenames in os.walk(ROOT):
        dirnames[:] = [d for d in dirnames if d not in SKIP_DIRS]
        for fn in filenames:
            if os.path.splitext(fn)[1].lower() not in EXTS:
                continue
            p = os.path.join(dirpath, fn)
            rel = os.path.relpath(p, ROOT)
            b = open(p, "rb").read()
            enc, text = detect(b)
            if text is None:
                bad.append((rel, enc, "无法解码"))
                continue
            if is_mojibake(text):
                bad.append((rel, enc, "内容疑似 mojibake（需人工确认）"))
                continue
            if enc == "UTF-8":
                continue
            changed.append((rel, enc))
            if apply:
                try:
                    with open(p, "w", encoding="utf-8", newline="") as f:
                        f.write(text)
                except PermissionError:
                    print("   [跳过] %s 正被占用" % rel)
                    changed.pop()
                    bad.append((rel, enc, "文件被占用，稍后重跑"))

    print("需要转码的文件：%d" % len(changed))
    for rel, enc in changed:
        print("   %-58s %s -> UTF-8" % (rel[:58], enc))
    if bad:
        print("\n需要人工确认：%d" % len(bad))
        for rel, enc, why in bad:
            print("   %-58s %-14s %s" % (rel[:58], enc, why))
    if not apply and changed:
        print("\n（只报告。加 --apply 实际改写）")


if __name__ == "__main__":
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    main()
