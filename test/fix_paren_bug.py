#!/usr/bin/env python3
"""修复合并代码 (PR #2) 中的两处括号缺失编译错误。

1. raw_storage_worker.hpp: convertedFileName 的 path 除法表达式少一个右括号,
   .string() 错误地作用在 basic_string 上:
       (dir.filename() / (base + ext)).string();   <-- 错
       (dir.filename() / (base + ext))).string();  <-- 对
2. event_monitor.hpp: storageWorker_ 初始化表达式少一个右括号,
   5 个左括号只闭合 4 个, 导致后续成员初始化被当作函数调用,
   连锁产生 "expression cannot be used as a function" 与
   "expected '{' at end of input" 错误。

补丁幂等: 已修复的文件自动跳过。
"""
import pathlib
import sys

PATCHES = [
    (
        "include/raw_storage_worker.hpp",
        '? imageExtension(job.imageFormat) : ".pcd")).string();',
        '? imageExtension(job.imageFormat) : ".pcd"))).string();',
    ),
    (
        "include/event_monitor.hpp",
        '"max_pending_storage_jobs", 20)))),',
        '"max_pending_storage_jobs", 20))))),',
    ),
]

ok = True
for path_text, old, new in PATCHES:
    path = pathlib.Path(path_text)
    source = path.read_text(encoding="utf-8")
    if new in source:
        print(f"SKIP (already patched): {path_text}")
        continue
    count = source.count(old)
    if count != 1:
        print(f"ERROR: {path_text} expected 1 occurrence, found {count}", file=sys.stderr)
        ok = False
        continue
    path.write_text(source.replace(old, new), encoding="utf-8")
    print(f"PATCHED: {path_text}")

sys.exit(0 if ok else 1)
