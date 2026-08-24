# DataCache 使用指南 — 跟着数据走一遍

本文面向**使用者和运维者**：不关心内部实现，只想知道系统怎么用、每个模块帮你做什么、
数据从传感器到你手上经历了什么。构建命令、配置键的完整清单见 [README.md](README.md)；
内部设计原理见 [ARCHITECTURE.md](ARCHITECTURE.md)。

---

## 1. 这个系统是做什么的？

一句话：**像行车记录仪的"事件回放"功能**——平时相机和雷达的数据像流水一样经过内存、
只保留最近一小段、并不占硬盘；一旦发生碰撞、急刹等事件，系统立刻把**事件前后的
完整数据片段**打包压缩存到硬盘，随后转换成分析工具直接能用的格式，并可自动回传到
远端服务器。你随时可以用配套工具检查数据是否完好、导出成图片和点云文件。

它解决的核心问题是：车上数据量巨大，不可能全程录制存储；但事故和异常发生的时间
无法预知。持续缓存 + 事件定格，就是"既不漏掉关键时刻，又不存无用数据"。

---

## 2. 五分钟上手

```bash
# 1. 构建后启动全套系统（缓存节点 + 相机 + 雷达仿真 + 触发器）
ros2 launch datacache datacache.launch.py

# 2. 触发一次"碰撞"事件（录制事件前 5 秒 + 事件后 10 秒）
ros2 service call /request_trigger datacache/srv/EventTrigger "{event_name: 'collision'}"

# 推荐的按键方式（另开终端）：c=碰撞，b=急刹，m=手动采集，q=退出
ros2 run datacache keyboard_trigger_node

# 3. 查看产物（目录名 = 事件名_触发时刻）
ls records/collision_*/

# 4. 校验数据完好性（损坏会返回退出码 2）
ros2 run datacache record_reader records/collision_*/ --verify
```

触发命令会立即返回一条消息，告诉你事件是否被受理，以及当时各传感器是否正常，例如：

```
Event 'collision' accepted for storage [camera: ok, lidar: ok]
```

如果看到 `lidar: STALE 2.3s`，说明触发时雷达已 2.3 秒没有数据——这次录制里雷达
片段可能不完整，需要排查设备。

按键方式走统一事件总线：`keyboard_trigger_node` 发布 `/event_signal`，
`event_router_node` 负责重复事件过滤和冷却，再调用缓存节点 RPC。按键终端会连续显示
事件被受理、落盘完成和 RPC 上传完成；其他系统也可以订阅 `/event_status` 获取同样状态。

> 没有真实相机？用合成视频源替代（30fps 移动图案，专为测试准备）：
> `python3 test/test_camera_publisher.py`，或者只跑雷达链路也完全可以。

---

## 3. 数据的完整生命周期

一条传感器数据从诞生到最终被分析使用，会经历 7 个阶段。每个阶段由一个模块负责，
下图就是全文的地图，后续章节按这个顺序逐一展开：

```
 ①产生        ②暂存         ③配对记账        ④事件定格
┌─────────┐  ┌──────────┐  ┌─────────────┐  ┌──────────────┐
│camera_node│▶│ DataBuffer│▶│ Approximate  │  │ EventMonitor  │◀── /request_trigger
│lidar_sim │  │ (内存环形  │  │ Synchronizer │  │ (剪出前后窗口) │     (event_trigger_node)
│  _node   │  │  缓存)    │  │ + PairIndex  │  └──────┬───────┘
└─────────┘  └──────────┘  └─────────────┘         │ ⑤压缩入库
                                               ┌────▼───────────────┐    ┌──────────────┐
                                               │ RawStorageWorker    │───▶│DiskSpaceManager│
                                               │ .bin.zst + manifest │    │ (磁盘管家)      │
                                               │ + jpg/pcd + .complete│  └──────────────┘
                                               └────┬───────────────┘
                                              ⑥寄出签收│          ⑦验货取用
                                               ┌────▼──────┐  ┌───────────────┐
                                               │UploadWorker│  │ record_reader  │
                                               │ RPC 回传   │  │ 校验/导出/统计  │
                                               └───────────┘  └───────────────┘
```

---

## 4. 阶段①：数据产生 — 传感器节点

| 节点 | 功能 | 备注 |
|---|---|---|
| `camera_node` | 从 USB 相机采图，以 30fps 发布到 `/image_raw` | 分辨率、设备号、帧率可用启动参数调 |
| `lidar_sim_node` | 读取一个 PCD 点云文件，以 10Hz 循环发布到 `/point_cloud` | 用真实雷达时替换成对应驱动节点即可，系统不区分来源 |

两个节点只是数据源头，可以被任何发布同类型话题的节点替换——DataCache 只订阅
`/image_raw` 和 `/point_cloud`，谁发布都一样。这是它作为"缓存回流层"的可组合性所在。

## 5. 阶段②：暂存 — DataBuffer（内存环形缓存）

**功能：让"过去"停留在手里一小会儿。**

所有进来的数据先存入内存缓存，缓存有两个上限，任一触顶就自动淘汰最旧数据：

- 条数上限（`buffer_size`，默认每类传感器 1000 条）；
- 年龄上限（`buffer_duration_seconds`，默认 30 秒）——按数据自带的时间戳计算，
  而不是收到的时间。

这正是"事件前 5 秒"能拿得出来的原因：事件发生时，那 5 秒的数据还躺在缓存里。
想录更长的 pre 窗口，就要把年龄上限调得比窗口更长（默认 30 秒 > 默认 pre 5 秒，够用）。

## 6. 阶段③：配对记账 — 同步器与账本

**功能：告诉分析人员"哪一帧照片和哪一帧点云是同一时刻的"。**

相机 30fps、雷达 10Hz，两路数据的时间戳几乎不可能完全相等。
`ApproximateSynchronizer` 按"时间差不超过 `sync_tolerance_ms`（默认 20 毫秒）"
把两侧消息配成对；配不上的单独记录原因（超出容差 / 队列满 / 事件边界清算）。

`PairIndex` 是这些配对结果的**账本**，每个事件落盘时会输出为 `pairs.csv`：
分析时能直接看到每对的时间差（`time_diff_ns`），以及哪些帧只有单侧数据、为什么。

另有一个严格模式 `sync_required_for_recording`：开启后，如果事件前窗口内连一对
成功配对的数据都没有，直接**拒绝录制**并告知触发方——避免存下一份注定无法做
多传感器融合分析的数据。默认关闭。

## 7. 阶段④：事件定格 — EventMonitor（窗口切分与调度）

**功能：事件发生的那一瞬间，决定"存哪些、存多久"。**

收到触发（如 `collision`）后，它按配置剪出两段：

- **pre 窗口**：事件前 `pre_time` 秒（collision 默认 5 秒）——数据已在缓存里，
  立即取出入库；
- **post 窗口**：事件后 `post_time` 秒（collision 默认 10 秒）——数据还没发生，
  挂一个后台任务，由内部定时器每 50ms 检查一次"够了没有"，到期再取一次入库。

两个保护机制值得知道：

- **窗口边界按传感器时间计算**，不受车载电脑与传感器时钟偏差的影响，不会切错片；
- 若传感器时间停滞（设备假死），post 任务也不会永远卡住——超过 `post_time + 5 秒`
  的墙钟宽限后就按现有数据收尾。

每种事件可有独立配置：窗口多长、录不录相机、录不录雷达
（如 `event_hard_brake_record_lidar=false` 表示急刹事件只录相机）。
触发瞬间如果系统忙不过来（存储队列满、并发事件过多），事件会被**明确拒绝**而不是
悄悄存一半——返回消息会告诉你原因。

配套的 `SensorWatchdog`（看门狗）持续盯着"每个传感器多久没来数据了"，
传感器断流立刻报错、恢复时提示，状态附加在每次触发响应里（见第 2 节示例）。

## 8. 阶段⑤：压缩入库 — RawStorageWorker（后台落盘）

**功能：把窗口内的数据打包成规整的事件目录，一个事件一个包裹。**

所有费时操作——序列化、zstd 无损压缩、图片/点云格式转换、写硬盘——都在独立后台
线程完成，绝不拖慢数据接收。每个事件目录长这样：

```
records/collision_0_1700000000123456789/
├── manifest.csv        # 清单：每条数据的文件名、编码、时间戳
├── pairs.csv           # 配对账本（阶段③的输出）
├── manifest.sha256     # 全部数据文件的 SHA-256 完整性清单
├── camera_<时间戳>.bin.zst   # 原始数据，zstd 无损压缩
├── lidar_<时间戳>.bin.zst
├── images/camera_<时间戳>.jpg    # 即开即用的分析副本
├── pointclouds/lidar_<时间戳>.pcd
└── .complete           # "本目录已写完"的标记（隐藏文件）
```

- `.bin.zst` 是**原始数据的无损压缩**（ROS2 消息序列化 + zstd），日后可完整还原，
  适合归档与回放；`images/`、`pointclouds/` 是**有损/转换副本**（jpg 质量 90 可调），
  适合直接拖进标注、可视化工具。两套并存，各取所需。
- `manifest.csv` 是目录的"总账"，记录每条数据的一切索引信息。
- 写文件的方式保证**不会留下半个文件**（先写临时名再原子改名），压缩数据自带
  校验和，磁盘静默损坏在回读时能被发现。
- `.complete` 只在所有清单写完之后才出现——它出现即代表这个包裹内容完整。
- 受理后先出现 `.pending`；任一 pre/post 批次或必需记录失败会转成 `.failed`。
  进程重启时会自动对账崩溃遗留的 `.pending`。

## 9. 阶段⑥：磁盘管家 — DiskSpaceManager

**功能：保证硬盘永远不会被录满。** 三道防线：

1. **写前检查**：剩余空间低于 `disk_min_free_mb`（默认 512MB）时先触发清理，
   还不够就拒写并报错（事件显式失败，而不是把盘写爆拖垮整车系统）；
2. **过期清理**：事件目录超过 `retention_days`（默认 30 天）自动删除；
3. **总量控制**：records 总量超过 `retention_max_capacity_mb`（默认 10GB）时
   **从最旧的删起**。

它只识别"本系统命名规则"的事件目录（`事件名_时间戳`），你手工放进 records 的
其他文件不会被误删。

## 10. 阶段⑦a：寄出签收 — UploadWorker（自动回传）

**功能：快递员——把写完的包裹自动寄给远端服务器。**

后台每 2 秒扫描一次：发现带 `.complete` 且还没寄过的目录，就把整个目录通过
`datacache/srv/UploadStore` 服务分块发送到 `upload_service_name`。一次目录传输依次调用
BEGIN（声明文件数/总字节数和 transfer_id）→ FILE_CHUNK（逐文件分块）→
END（接收端校验文件数、字节数与 SHA-256 后确认）。行为规则：

- 成功 → 写 `.uploaded` 标记（记录寄往哪、几个文件）；**本地数据保留不删**，
  磁盘回收永远是磁盘管家的职责；
- 失败 → 自动重试，间隔指数拉长（15s 起，最多翻到 64 倍）；连续失败超过
  `upload_max_retries`（默认 5 次）→ 写 `.upload_failed`，默认 30 分钟后重新排队。

默认关闭。打开方法与配套的测试接收端：

```bash
# 终端 A：起包内测试接收节点（END 时验证 SHA-256 清单）
ros2 run datacache upload_receiver_node upload_inbox /upload_store

# 终端 B：配置文件里 upload_enabled=false 改为 true 后启动
ros2 run datacache datacache_node --ros-args -p config_path:=config.txt
```

> 注意：发送端与接收端必须处于能够互相发现的 ROS2 DDS 域。跨网段/车云部署需要
> DDS Router、VPN 或网关，并应按环境启用 DDS Security；服务名必须与
> `upload_service_name` 一致。

## 11. 阶段⑦b：验货取用 — record_reader（回读工具）

**功能：不启动整个系统，直接检查和使用已落盘的数据。**

```bash
# 概要统计：多少条相机/雷达记录、各多大、什么编码
ros2 run datacache record_reader records/collision_0_1700000000123456789

# 完整性校验：文件存在 → 解压 → 反序列化 → 字段自洽，逐条过一遍
# 任何一条损坏/缺失/清单撕裂 → 退出码 2（可直接用于脚本判断）
ros2 run datacache record_reader records/collision_0_1700000000123456789 --verify

# 导出为 png / pcd 文件到指定目录
ros2 run datacache record_reader records/collision_0_1700000000123456789 --export out/

# 只看相机、只看前 10 条
ros2 run datacache record_reader records/... --sensor camera --limit 10
```

数据一旦离开本系统（拷贝、传输、归档），用 `--verify` 做一次全量校验是最可靠的
完好性凭证——校验逻辑与写入共用同一套代码，"写得出就读得回"。

---

## 12. 按目标查配置

配置文件 `config.txt`，`key=value`，`#` 为注释。完整键表见 README，这里按
"你想做什么"组织：

| 你想… | 改这些键 |
|---|---|
| 缓存保留更久（想录更长的 pre 窗口） | `buffer_duration_seconds`（须大于最大 pre 窗口）、`buffer_size` |
| 调整某事件的录制窗口 | `event_collision_pre_time` / `event_collision_post_time`（其他事件同理） |
| 某事件只录一种传感器 | `event_<名称>_record_camera` / `record_lidar` |
| 增删事件类型 | `enable_collision_event` / `enable_hard_brake_event`（新事件名需在代码注册） |
| 提高配对成功率 | `sync_tolerance_ms`（放宽容差）；排查单侧丢帧看 `pairs.csv` 的 reason 列 |
| 没有配对就不录制 | `sync_required_for_recording=true`（依赖 `sync_enabled=true`） |
| 更小体积 / 更快写入 | `compression_level`（1–19，越高越小越慢）、`conversion_enabled=false` 可关转换副本 |
| 同时保留未压缩原始 .bin | `keep_raw_after_compression=true` |
| 调整导出图片质量 | `image_format`（jpg/png）、`image_quality`（1–100） |
| 磁盘保护阈值与保留期 | `disk_min_free_mb`、`retention_days`、`retention_max_capacity_mb` |
| 开启自动回传 | `upload_enabled=true` + `upload_service_name`；重试节奏 `upload_max_retries` / `upload_retry_backoff_ms` |
| 传感器断流的判定灵敏度 | `watchdog_stale_timeout_ms`（全局）、`watchdog_camera_stale_timeout_ms`（单传感器覆盖） |
| 落盘位置 | `record_directory` |
| 定时自动触发事件（测试用） | launch 参数 `event_interval:=30`（每 30 秒自动触发一次默认事件） |

---

## 13. 常见问题排查

**触发被拒绝，返回消息说 "was not accepted"？**
按可能性依次检查：事件名是否拼写正确且对应的 `enable_*_event` 已开启；
存储队列是否被占满（`max_pending_storage_jobs`，可调大）；
并发事件是否过多（`max_active_event_captures`）；
严格模式下 pre 窗口是否无配对（看 `pairs.csv` 或放宽 `sync_tolerance_ms`）。

**触发返回了消息里的 `STALE` 是什么意思？**
该传感器已多秒没有新数据（默认 1 秒判静默）。本次录制中该传感器的片段可能不完整
甚至缺失。排查设备连接与驱动，恢复后会有 `recovered` 日志。

**事件目录里没有 `.complete`？**
说明写入没有正常完成：最常见是触发时磁盘剩余空间不足（写前检查拒绝），
或 post 窗口数据入队时存储队列已满。查看目录中的 `.failed` 可确认失败终态；
崩溃遗留的 `.pending` 会在下次启动时自动转成 `.failed`。这类目录不会被回传；
反复出现则调大 `max_pending_storage_jobs` 或清理磁盘。

**出现 `.upload_failed` 怎么办？**
一轮快速重试已耗尽。默认会在 `upload_failed_rescan_period_ms`（30 分钟）后自动
重新排队；设为 0 才是永久终态，此时确认接收端可达后删除该标记即可手动重试。

**`pairs.csv` 里大量 camera_only / lidar_only？**
两路数据时间戳经常对不上：检查两传感器的时钟同步（NTP/PTP）、帧率是否悬殊、
`sync_tolerance_ms` 是否过小。队列满丢弃（"synchronizer queue full"）则说明
一侧持续快于另一侧。

**怎么快速验证整套系统？**
```bash
bash tools/smoke_test.sh    # 自动缩短窗口：触发→落盘→校验→导出→上传闭环一遍过
```

**数据已被拷贝到别处，如何确信没坏？**
在任意装有本包的机器上 `record_reader <目录> --verify`，退出码 0 即完整。
zstd 帧自带内容校验和，磁盘静默位腐也会被查出。

---

## 14. 三份文档怎么选

| 文档 | 适合谁 | 内容 |
|---|---|---|
| [USER_GUIDE.md](USER_GUIDE.md)（本文） | 使用者、运维 | 功能说明、数据生命周期、按目标查配置、故障排查 |
| [README.md](README.md) | 首次搭建 | 构建、运行命令、完整配置键表 |
| [ARCHITECTURE.md](ARCHITECTURE.md) | 开发、维护 | 模块内部机制、线程/时钟域设计、迭代史 |
