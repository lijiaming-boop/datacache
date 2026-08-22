#!/usr/bin/env python3
"""校验 datacache 事件录制产物。

对 records/ 下的每个事件目录执行结构化断言:
  - manifest.csv 存在、表头正确、含 sensor 记录行
  - 记录文件存在, zstd 编码的文件具有 zstd 魔数 (28 B5 2F FD)
  - manifest 的 converted_file 列指向真实存在的转换产物
  - images/*.jpg 与 pointclouds/*.pcd 转换产物存在
  - pairs.csv 存在且含 matched 行(时间同步链路工作)
  - 事件级传感器选择生效: hard_brake 不应录制 lidar(config 关闭)

退出码: 0 全部通过, 1 存在失败项。
"""
import csv
import pathlib
import sys

RECORDS_DIR = pathlib.Path('records')
ZSTD_MAGIC = b'\x28\xb5\x2f\xfd'
MANIFEST_HEADER = ['sensor', 'timestamp', 'file', 'encoding', 'converted_file']
PAIRS_HEADER = ['pair_id', 'status', 'camera_timestamp', 'lidar_timestamp',
                'time_diff_ns', 'reason']

passed: list[str] = []
failed: list[str] = []


def check(condition: bool, description: str) -> bool:
    (passed if condition else failed).append(description)
    return condition


def read_csv(path: pathlib.Path) -> list[dict]:
    with path.open(newline='') as handle:
        reader = csv.reader(handle)
        rows = list(reader)
    if not rows:
        return []
    return [dict(zip(rows[0], row)) for row in rows[1:]]


def validate_manifest_header(directory: pathlib.Path) -> None:
    header_path = directory / 'manifest.csv'
    if not check(header_path.is_file(), f'{directory.name}: manifest.csv 存在'):
        return
    with header_path.open(newline='') as handle:
        header = next(csv.reader(handle), [])
    check(header == MANIFEST_HEADER,
          f'{directory.name}: manifest.csv 表头正确 (实际: {header})')


def validate_event_directory(directory: pathlib.Path, expect_lidar: bool) -> None:
    name = directory.name
    validate_manifest_header(directory)

    manifest_path = directory / 'manifest.csv'
    if not manifest_path.is_file():
        return
    rows = read_csv(manifest_path)

    camera_rows = [r for r in rows if r.get('sensor') == 'camera']
    lidar_rows = [r for r in rows if r.get('sensor') == 'lidar']
    check(len(camera_rows) > 0, f'{name}: manifest 含 camera 记录 ({len(camera_rows)} 行)')
    if expect_lidar:
        check(len(lidar_rows) > 0, f'{name}: manifest 含 lidar 记录 ({len(lidar_rows)} 行)')
    else:
        check(len(lidar_rows) == 0, f'{name}: manifest 不含 lidar 记录 (事件级配置生效)')

    for row in rows:
        record_file = directory / row['file']
        if not check(record_file.is_file(), f'{name}: 记录文件存在 {row["file"]}'):
            continue
        if row['encoding'] == 'zstd':
            magic = record_file.read_bytes()[:4]
            check(magic == ZSTD_MAGIC, f'{name}: {row["file"]} 为有效 zstd 帧')
        elif row['encoding'] != 'raw':
            check(False, f'{name}: {row["file"]} 编码列异常 ({row["encoding"]})')
        converted = row.get('converted_file', '')
        if converted:
            check((directory / converted).is_file(),
                  f'{name}: converted_file 指向真实文件 {converted}')

    jpgs = list((directory / 'images').glob('*.jpg'))
    check(len(jpgs) > 0, f'{name}: images/ 含 jpg 转换产物 ({len(jpgs)} 个)')
    if expect_lidar:
        pcds = list((directory / 'pointclouds').glob('*.pcd'))
        check(len(pcds) > 0, f'{name}: pointclouds/ 含 pcd 转换产物 ({len(pcds)} 个)')
    else:
        pcd_dir = directory / 'pointclouds'
        check(not pcd_dir.exists() or not list(pcd_dir.glob('*.pcd')),
              f'{name}: 无 pcd 产物 (lidar 未录制)')

    pairs_path = directory / 'pairs.csv'
    if check(pairs_path.is_file(), f'{name}: pairs.csv 存在'):
        pair_rows = read_csv(pairs_path)
        matched = [r for r in pair_rows if r.get('status') == 'matched']
        check(len(matched) > 0,
              f'{name}: pairs.csv 含 matched 同步对 ({len(matched)} 条)')
        for row in matched:
            diff = abs(int(row['time_diff_ns']))
            check(diff <= 20_000_000,
                  f'{name}: matched 对时差在 20ms 容差内 ({diff / 1e6:.2f}ms)')


def main() -> int:
    if not check(RECORDS_DIR.is_dir(), 'records/ 目录存在'):
        report()
        return 1

    collision_dirs = sorted(RECORDS_DIR.glob('collision_*'))
    hard_brake_dirs = sorted(RECORDS_DIR.glob('hard_brake_*'))
    check(len(collision_dirs) > 0, f'存在 collision 事件目录 ({len(collision_dirs)} 个)')
    check(len(hard_brake_dirs) > 0, f'存在 hard_brake 事件目录 ({len(hard_brake_dirs)} 个)')

    for directory in collision_dirs:
        validate_event_directory(directory, expect_lidar=True)
    for directory in hard_brake_dirs:
        validate_event_directory(directory, expect_lidar=False)

    report()
    return 0 if not failed else 1


def report() -> None:
    print(f'\n===== 校验结果: {len(passed)} 通过, {len(failed)} 失败 =====')
    if failed:
        print('\n失败项:')
        for item in failed:
            print(f'  [FAIL] {item}')
    print()
    print('通过项:')
    for item in passed:
        print(f'  [PASS] {item}')


if __name__ == '__main__':
    sys.exit(main())
