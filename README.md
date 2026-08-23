# DataCache — 车载数据回流系统

ROS2 事件触发的传感器数据缓存与回流系统：相机/激光雷达数据持续进入内存环形缓存，
事件（碰撞、急刹等）触发时把事件前后窗口的数据压缩落盘，后台转换为分析友好格式，
并可整目录回传到远端接收服务。配套回读工具校验/导出落盘数据，形成完整闭环。

```
┌─────────────┐   /image_raw   ┌──────────────────────────────────────────┐
│ camera_node │───────────────▶│                                          │
└─────────────┘                │  DataBuffer (按条数+年龄的环形缓存)        │
┌──────────────┐ /point_cloud  │      │                                   │
│ lidar_sim_   │──────────────▶│  ApproximateSynchronizer (±tolerance 配对)│
│ node         │                │      │ matched / single-sided               │
└──────────────┘                │  PairIndex (同步账本)                    │
                                │      │                                   │
┌─────────────────┐  service    │  EventMonitor (pre/post 窗口, 调度器)     │
│ event_trigger_  │──/trigger──▶│      │ enqueue                            │
│ node            │  _event     │  RawStorageWorker (后台线程)              │
└─────────────────┘             │   ├ CDR 序列化 → .bin                    │
                                │   ├ zstd 压缩    → .bin.zst              │
                                │   ├ 格式转换    → images/*.jpg           │
                                │   │               pointclouds/*.pcd      │
                                │   ├ manifest.csv / pairs.csv             │
                                │   └ .complete 标记 ──▶ UploadWorker      │
                                └──────────────────────────────────────────┘
                                                          │ HTTP multipart
                                                          ▼
                                                 接收端 (如 tools/mock_server.py)
```

## 构建（Ubuntu 24.04 / ROS2 Jazzy）

```bash
sudo apt install ros-jazzy-desktop ros-jazzy-cv-bridge ros-jazzy-pcl-conversions \
                 libzstd-dev libcurl4-openssl-dev
cd datacache
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 运行

```bash
# 终端 1: 全套系统(缓存节点 + 相机 + 雷达仿真 + 触发器)
ros2 launch datacache datacache.launch.py

# 终端 2: 触发一次碰撞事件(5 s pre + 10 s post 窗口)
ros2 service call /request_trigger datacache/srv/EventTrigger "{event_name: 'collision'}"
```

事件数据落在运行目录的 `records/<event>_<ns>/` 下：

```
records/collision_1700000000123456789/
├── manifest.csv          # sensor,timestamp,file,encoding,converted_file
├── pairs.csv             # pair_id,status,camera_timestamp,lidar_timestamp,time_diff_ns,reason
├── camera_<ns>.bin.zst   # zstd 压缩的 CDR 序列化 sensor_msgs/Image
├── lidar_<ns>.bin.zst    # zstd 压缩的 CDR 序列化 sensor_msgs/PointCloud2
├── images/camera_<ns>.jpg        # 后台转换的分析友好副本
├── pointclouds/lidar_<ns>.pcd
├── .complete             # 事件目录写入完成标记(上传模块据此启动回传)
└── .uploaded             # 上传成功后出现(如启用上传)
```

## 回读工具 record_reader

```bash
# 概要: manifest/pairs 条数、每条记录的大小与编码
ros2 run datacache record_reader records/collision_1700000000123456789

# 完整性校验: 文件存在 → zstd 解压 → CDR 反序列化 → 字段自洽; 损坏返回退出码 2
ros2 run datacache record_reader records/collision_1700000000123456789 --verify

# 导出为 png / pcd
ros2 run datacache record_reader records/collision_1700000000123456789 --export out/

# 只看相机, 前 10 条
ros2 run datacache record_reader records/collision_1700000000123456789 --sensor camera --limit 10
```

## 上传/回传

`RawStorageWorker` 写完一个事件目录的最后一批数据后写入 `.complete` 标记；
`UploadWorker` 后台线程周期扫描 `record_directory`，把带 `.complete` 且未上传的
目录通过 HTTP multipart POST 整目录发到 `upload_url`。成功写 `.uploaded`，
失败按指数退避重试，超过 `upload_max_retries` 次写 `.upload_failed`。本地数据
不删除，磁盘回收由保留策略负责。

验证闭环可使用自带的 mock 接收端（纯 Python 标准库）：

```bash
# 终端 A
python3 tools/mock_server.py 8080 upload_inbox

# 终端 B: 打开上传开关并启动
sed -i 's/upload_enabled=false/upload_enabled=true/' config.txt
ros2 run datacache datacache_node --ros-args -p config_path:=config.txt
# 触发事件后, mock 端会打印收到的文件清单(md5), 事件目录出现 .uploaded
```

## 单元测试

```bash
colcon test && colcon test-result --verbose
```

覆盖模块：ConfigManager（解析/默认值）、DataBuffer（数量与年龄驱逐/时间范围）、
PairIndex（账本/裁剪）、ApproximateSynchronizer（配对/丢弃/冲刷）、
record_io（写→读回环 + 损坏/撕裂行检测）、UploadWorker（候选扫描/HTTP 上传/标记/
后台循环/重试耗尽与退避）、EventMonitor（pre/post 窗口切分/调度到期/预留回滚/
并发上限）、DiskSpaceManager（天数与容量清理/写前检查/节流）。

## 端到端冒烟验证

在已 source ROS2 与 install 环境的 shell 中执行（自动缩短事件窗口加快验证）：

```bash
bash tools/smoke_test.sh
```

脚本会启动 mock 接收端、雷达仿真与缓存节点，触发一次 collision 事件，
然后依次验证：事件目录产物 → `record_reader --verify` 全量校验 → `--export`
导出 → 上传闭环（`.uploaded` 标记 + 接收端落盘 + md5 清单）。

## 配置参考（config.txt）

| 键 | 默认 | 说明 |
|---|---|---|
| `buffer_size` | 1000 | 每传感器类型的缓存条数上限 |
| `buffer_duration_seconds` | 30 | 缓存年龄上限（按最新传感器时间戳） |
| `sync_enabled` | true | 是否启用近似时间同步 |
| `sync_queue_size` | 100 | 同步器每侧队列上限 |
| `sync_tolerance_ms` | 20 | 配对容差 |
| `sync_required_for_recording` | false | 严格模式：pre 窗口无 matched 配对则拒绝录制 |
| `max_pending_storage_jobs` | 20 | 存储队列上限 |
| `max_active_event_captures` | 16 | 并发 post 窗口捕获上限 |
| `event_scheduler_period_ms` | 50 | post 窗口调度周期 |
| `sensor_stall_grace_ms` | 5000 | 传感器时间戳停滞时的墙钟兜底宽限 |
| `watchdog_enabled` | true | 接收时间新鲜度看门狗 |
| `watchdog_check_period_ms` | 500 | 看门狗轮询周期 |
| `watchdog_stale_timeout_ms` | 1000 | 判定静默的超时（可按传感器覆盖） |
| `compression_enabled` | true | zstd 压缩落盘记录 |
| `compression_level` | 3 | zstd 级别 1–19 |
| `keep_raw_after_compression` | false | 压缩后是否保留未压缩 .bin |
| `conversion_enabled` | true | 后台格式转换 |
| `image_format` / `image_quality` | jpg / 90 | 图像转换格式与质量 |
| `pointcloud_format` | pcd | 点云转换格式（仅 pcd） |
| `disk_min_free_mb` | 512 | 写前最低剩余空间（0 禁用） |
| `retention_days` | 30 | 事件目录保留天数（0 禁用） |
| `retention_max_capacity_mb` | 10240 | records 总量上限（0 禁用） |
| `disk_cleanup_interval_seconds` | 60 | 清理节流间隔 |
| `upload_enabled` | false | 是否启用回传 |
| `upload_url` | — | 接收端地址，如 http://host:8080/upload |
| `upload_scan_period_ms` | 2000 | 扫描周期 |
| `upload_timeout_s` | 30 | 单次上传超时 |
| `upload_max_retries` | 5 | 重试次数上限（超过标记 .upload_failed） |
| `upload_retry_backoff_ms` | 15000 | 首次重试退避（指数翻倍，封顶 64 倍） |
| `enable_collision_event` 等 | true | 事件注册开关 |
| `event_<name>_pre_time/post_time` | 5/5 | 每事件窗口秒数 |
| `event_<name>_record_camera/lidar` | true | 每事件传感器选择 |
| `record_directory` | records | 落盘根目录 |

## 设计要点

- **异步落盘**：订阅回调只入队，序列化/zstd/JPEG/PCD 全部在单个后台线程执行；
  存储队列满时通过预约机制优先保证 post 窗口数据的槽位。
- **原子 + 落盘写**：所有记录文件先写 `.tmp`、fsync 后再 rename（并同步父目录项），
  进程崩溃不留半文件，掉电时 rename 也不会先于数据块可见；`.complete` 标记在
  manifest/pairs 刷盘之后写入。zstd 帧携带 XXH64 内容校验和，静默位腐在解压时
  即可被发现。
- **O(1) 摊还热路径**：DataBuffer 按传感器类型分队列、队首弹出淘汰；
  同步器前端贪心配对、锁外派发回调；PairIndex 用 deque 裁剪——每条消息的
  处理成本不随缓存/账本规模增长。
- **传感器时间域**：事件窗口边界用缓存内最新 `header.stamp` 计算，避免节点时钟
  与传感器时钟偏移导致切片错位；post 窗口另有墙钟兜底防传感器停滞钉死任务；
  pre/post 批次在 eventTime 边界去重。
- **磁盘安全**：写前剩余空间检查 + 天数/容量双保留策略，超限时从最旧事件目录
  开始回收。
- **写读闭环**：`record_io.hpp` 同时供 CLI 与单测使用，落盘格式（CDR+zstd）有
  回读实现与损坏检测保障；崩溃残留的撕裂 CSV 行按问题上报而非抛异常。

## 已知限制

- 上传为自定义 multipart 格式，生产环境需接收端按相同约定解析（参考 mock_server.py）。
- post 窗口数据入队失败（队列满）时仅记录错误，该事件目录不会有 `.complete` 标记，
  也不会被回传。
- `pointcloud_format` 仅支持 pcd；视频片段为逐帧图像，无 h264/h265 编码。
- 相机节点依赖 GUI 环境的 USB 设备；无相机时可只跑 lidar_sim 链路。
- `.upload_failed` 为永久终态，当前无自动/命令行重试入口，需人工删除标记后
  等待下个扫描周期。
