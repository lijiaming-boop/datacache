#!/usr/bin/env python3
"""扫描 C++ 头文件的括号/花括号平衡 (剥离字符串、字符字面量与注释)。

用于在编译前快速定位类似 PR #2 那种缺失右括号的语法错误。
用法: python3 check_paren_balance.py include/*.hpp src/*
"""
import pathlib
import sys


def strip_code(source: str) -> str:
    """移除注释与字符串/字符字面量, 只保留结构性代码。"""
    out = []
    i, n = 0, len(source)
    while i < n:
        ch = source[i]
        nxt = source[i + 1] if i + 1 < n else ''
        if ch == '/' and nxt == '/':          # 行注释
            while i < n and source[i] != '\n':
                i += 1
        elif ch == '/' and nxt == '*':        # 块注释
            i += 2
            while i + 1 < n and not (source[i] == '*' and source[i + 1] == '/'):
                i += 1
            i += 2
        elif ch == '"' or ch == "'":          # 字符串/字符字面量
            quote = ch
            i += 1
            while i < n and source[i] != quote:
                if source[i] == '\\':
                    i += 1
                i += 1
            i += 1
        else:
            out.append(ch)
            i += 1
    return ''.join(out)


def main() -> int:
    bad = 0
    for arg in sys.argv[1:]:
        path = pathlib.Path(arg)
        if not path.is_file():
            continue
        code = strip_code(path.read_text(encoding='utf-8'))
        # 尖括号忽略 (模板), 只查 () {} []
        for opener, closer in [('(', ')'), ('{', '}'), ('[', ']')]:
            opens, closes = code.count(opener), code.count(closer)
            if opens != closes:
                print(f"UNBALANCED {path}: '{opener}{closer}' {opens} 开 / {closes} 闭 "
                      f"(差 {opens - closes:+d})")
                bad += 1
    if not bad:
        print("All files balanced.")
    return 1 if bad else 0


if __name__ == '__main__':
    sys.exit(main())
