# DataCache 生产化改进方案

本文承接差距分析（见第 11 节对照表），对每项不足给出**可落地的解决方案**：

> 2026-08-23 实施状态：2.1/2.2/2.3、3.2、4.3 已完成；4.2 已完成
> `transfer_id` 与安全整段重传，QUERY 偏移协商仍待实现；冒烟闭环已进入 CI。
方案思路、改动落点（精确到模块/文件）、验收标准与权衡。供评审与排期使用；
配套的实施顺序见第 10 节的四个里程碑。

三条原则贯穿所有方案：

1. **不破坏现有契约**：事件目录格式、标记文件状态机、`record_io` 写读闭环保持兼容，
   新能力以增量文件/字段的方式加入。
2. **每步可独立验证**：每个方案都附带验收标准，合入即生效，不依赖后续里程碑。
3. **最小改动优先**：先堵住静默数据丢失和工程基建，再做结构性演进（多传感器、
   MCAP 等），避免一步到位的大重写。

---

## 0. 方案总览

| 差距域 | 核心方案 | 里程碑 | 量级 |
|---|---|---|---|
| 工程基建 | GitHub Actions CI + LICENSE + 配置校验 | M0 | 小 |
| 静默数据丢失 | `.pending`/`.failed` 标记 + 启动对账 + 运维 CLI | M1 | 中 |
| 上传终态不可恢复 | 失败周期性重排队 + CLI 重试入口 | M1 | 小 |
| 事件风暴 | 冷却期 + 活跃窗口合并 | M1 | 小 |
| 车云安全 | DDS Security + 网络边界网关 + 清单哈希 | M2 | 中 |
| 回传断点 | RPC 已分块流式读取；补 transfer_id + 断点查询 | M2 | 中 |
| 容量与吞吐 | 字节级缓存预算 + 基准测试 + 可选存储线程池 | M2/M3 | 中 |
| 可观测性 | 指标主题 + 每事件元数据（event.json） | M2 | 中 |
| 生态互通 | record_reader 增加 MCAP 导出 | M3 | 中 |
| 扩展性 | 事件注册配置化、通用传感器通道、时钟偏移检测 | M3 | 大 |
| 测试深度 | 崩溃一致性注入、launch 测试、覆盖率门禁 | M0–M3 | 渐进 |

---

## 1. 工程基建（M0）

### 1.1 CI 流水线

**方案**：新增 `.github/workflows/ci.yml`，容器 `ros:jazzy-ros-base`（Ubuntu 24.04），
三个 job：

1. **build+test**：`colcon build` → `colcon test` → `colcon test-result`，
   失败即红。依赖用 apt 装 `libzstd-dev` 及 rosdep 解析。
2. **format**：仓库加 `.clang-format`（按现有代码风格生成一次后固化），
   CI 跑 `clang-format --dry-run --Werror` 只检查不重写。
3. **sanitizers**（可与 1 同 job 分步）：以
   `-DCMAKE_CXX_FLAGS="-fsanitize=address,undefined"` 重编并只跑单测；
   TSan 单独一步（与 ASan 互斥）。同步器、双 worker 这类并发模块从此有竞争检测兜底。

**落点**：新增 `.github/workflows/ci.yml`、`.clang-format`；CMakeLists 加
`DATACACHE_SANITIZERS` 选项包裹编译 flags。

**验收**：PR 上自动出现三个检查；人为注入一个数据竞争（临时脚本）TSan 能报。

**权衡**：GitHub-hosted runner 装 ROS2 每次约 5–8 分钟；可接受，量大后再上
自托管 runner 或 docker 缓存。

### 1.2 许可与元数据

**方案**：补 `LICENSE`（MIT，与 package.xml 一致）；package.xml 的 maintainer
换成真实姓名邮箱；加 `CHANGELOG.md`（Keep a Changelog 格式），此后每个合入
main 的 feature 分支在 CHANGELOG 的 Unreleased 节追加一行，发布时打
`git tag v*` 并升 package.xml 版本。

**验收**：`cat LICENSE` 非空；tag v1.1.0 存在且 CHANGELOG 有对应小节。

### 1.3 配置键校验（消除静默默认值）

**方案**：ConfigManager 增加两件事：

- **已知键清单**：新增 `config_keys.hpp` 集中定义全部合法键名（含事件级前缀的
  生成规则）。`loadConfig()` 结束时对文件里出现但清单外的键逐个 `WARN`：
  `Unknown config key 'buffer_duration_secondd' (typo? ignored)`。
- **严格模式**：`config_strict=true` 时未知键或值解析失败直接 `loadConfig` 返回
  false，节点拒绝启动（车载部署用）。

另将 `record_directory` 在节点启动时 `std::filesystem::absolute()` 解析并打日志，
消除"相对 CWD 散落各处"的问题。

**落点**：`config_manager.hpp/.cpp`、`datacache_node.cpp:loadConfiguration`。

**验收**：单测新增——未知键触发 WARN 回调、严格模式拒绝启动；
`config.txt` 写错键名时启动日志明确指出。

---

## 2. 静默数据丢失清零（M1，最高优先级的可靠性工作）

### 2.1 受理语义与崩溃恢复：`.pending` / `.failed` 标记

**现状**：事件被受理后若 post 批入队失败或进程崩溃，目录无 `.complete`、无任何
失败标记，30 天后被保留策略静默删除；触发方还以为已存档。

**方案**：把状态机从
`.tmp → .complete → .uploaded/.upload_failed` 扩展为：

```
接受事件时写 .pending (含事件名/触发时间/配置摘要)
  ├─ 写入全部完成 → 删除 .pending，写 .complete        （现有路径）
  ├─ post 批入队失败 / 磁盘不足整批丢弃 → 写 .failed（含原因）
  └─ 进程崩溃 → .pending 残留 → 启动对账处理（见 2.2）
```

具体改动：

- `EventMonitor::recordDataAroundEvent`：受理成功路径上先建目录并原子写
  `.pending`（复用 `writeAtomically`）；触发服务响应措辞改为
  `"accepted (persistence pending)"`，诚实反映语义。
- `RawStorageWorker::writeJob`：`prepareForWrite()` 失败或 manifest 打开失败等
  "整批丢弃"分支，若 `job.finalJob` 则写 `.failed reason=disk_full` 等。
  post 批 `enqueue` 返回 false 时（event_monitor.hpp:199）同样补写 `.failed reason=queue_full`。
- `finalJob` 成功路径：写 `.complete` 后删除 `.pending`。

**验收**：单测——队列打满触发事件，目录出现 `.failed` 而非空目录；
集成测试断言成功路径最终无 `.pending` 残留。

### 2.2 启动对账（Startup Reconciliation）

**方案**：`DataCacheNode` 构造时（上传器启动前）扫描 `record_directory` 一次：

| 发现 | 处理 |
|---|---|
| `.pending` 且无 `.complete` | 改写为 `.failed reason=crash_recovery`（pre 批数据可能已部分写入，目录保留供人工取舍） |
| `.complete` 无 `.uploaded`/`.upload_failed` | 不动——UploadWorker 扫描周期自然接手（现有行为已正确） |
| `.upload_failed` | 见 2.3 的重排队策略 |

实现为一个自由函数放 `record_io.hpp`（与 CLI 共用），节点调用。

**验收**：单测构造残留 `.pending` 目录 → 启动后变 `.failed`；
`.complete` 目录不被误动。

### 2.3 上传失败恢复：终态变"长周期重试 + 告警"

**方案**：

- UploadWorker 对 `.upload_failed` 目录按新配置 `upload_failed_rescan_period_ms`
  （默认 30 分钟，0 = 维持终态行为）重新入队重试，重试前清除标记。
- 新增运维 CLI `record_admin`（与 record_reader 同骨架）：

```
ros2 run datacache record_admin records/ reconcile        # 手动触发 2.2 的对账
ros2 run datacache record_admin records/ retry-upload X   # 等价于删 .upload_failed 但带日志
ros2 run datacache record_admin records/ mark-failed X --reason "..."
```

README"已知限制"中"需人工删除标记"的操作由此工具化并留痕。

**验收**：单测——设 1 秒 rescan 周期，失败目录在标记后被重新尝试；
`retry-upload` 子命令行为有测试覆盖。

### 2.4 事件风暴与重复触发

**方案**（两层，都可配置关闭）：

1. **冷却期**：`event_<name>_cooldown_seconds`（回退 `event_cooldown_seconds`，
   默认 0 不限制）。冷却期内同型事件触发直接拒绝，响应写明
   `"rejected: cooldown after previous '<name>' event"`。实现：EventMonitor 记录
   各事件名上次受理的墙钟时间（一个 `unordered_map` + 已有 captureMutex_ 足够）。
2. **窗口合并**：同型事件落在某活跃 post 窗口内时，不新建目录，而是把该
   `EventCaptureTask` 的 `endTime/wallDeadline` 顺延新的 post 长度，并在该目录
   `.pending` 里追加一条合并记录。批次边界去重机制
   （event_monitor.hpp:186 的 remove_if）改为记录每个任务
   `lastEnqueuedTimestamp`，post 收割时剔除 `<= lastEnqueuedTimestamp` 的记录，
   即可天然消除合并窗口的重叠写入。

**验收**：单测——冷却期内二次触发被拒；两事件间隔 2s 合并为单目录且 manifest
无重复时间戳行。

### 2.5 外围节点健壮性

- **camera_node**：打开设备失败不再抛异常退出，改为指数退避重试
  （参数 `reopen_backoff_ms`），恢复后自动续采；期间看门狗自然会报 STALE。
- **record_reader/`test_camera_publisher.py` 不动**。

**验收**：无设备环境下 camera_node 持续重试不退出（CI 可测）。

---

## 3. 车云链路安全（M2）

### 3.1 传输加密与认证

**现状变化**：回传已从 HTTP/libcurl 迁移为 `UploadStore` ROS2 RPC，因此原 TLS、
Bearer 与 curl 配置不再适用。

**方案**：生产 DDS 域启用 SROS2/DDS Security（身份认证、访问控制、加密），为
DataCacheNode 与接收节点签发独立身份，并把 `/upload_store` 的 call/execute 权限写入
治理策略。车云跨网段通过 DDS Router/VPN 或受控网关连接，不直接向公网暴露 DDS 发现。

**验收**：无授权身份无法发现或调用 `/upload_store`；授权发送端可完成回传；抓包中
不可读取 FILE_CHUNK 载荷；权限策略拒绝非回传节点执行服务。

### 3.2 数据完整性清单

**方案**：`finalJob` 时除 `.complete` 外再写 `manifest.sha256`：
对目录内每个数据文件（含转换副本）算 SHA-256，格式
`<sha256>  <relative_path>`。`upload_receiver_node` 在 END 发布暂存目录前验证清单。
上链签名（HMAC/车辆证书）作为后续可选增强，清单格式预留 `sig:` 行。

**验收**：传输途中篡改任一字节，接收端校验脚本报错并指出文件。

---

## 4. 传输效率与资源（M2）

### 4.1 分块流式回传（已完成，补性能验收）

**现状**：`transferDirectory` 已按文件流式读取，默认以 512 KiB FILE_CHUNK RPC
发送，接口把单块限制为 1 MiB；峰值负载内存不再随整目录大小增长。

**待办**：增加 200MB 合成目录基准，断言进程 RSS 增量 < 50MB，并测量不同 chunk
大小和 RTT 下的吞吐，决定是否需要多块流水线或异步批量确认。

### 4.2 RPC 协议 v2：传输身份 + 断点续传

**方案**：在接口中增加 `transfer_id` 与 QUERY/ABORT 操作。接收端按
`transfer_id + event_name + file_path` 保存已连续接收偏移；发送端重启后先 QUERY，
从接收端确认的偏移继续发送。END 仍以 manifest.sha256 校验后原子发布目录。

**验收**：中途 kill 发送端，重启后不重传已确认块，最终目录 SHA-256 全部一致；
陈旧 transfer_id 可按超时清理，不能覆盖另一个发送端正在进行的事务。

### 4.3 字节级缓存预算

**方案**：DataBuffer 构造参数与配置增加 `buffer_max_mb`（每传感器，0 = 不限）。
`addData` 时按 `std::visit` 计算消息字节数（Image 用 `data.size()`，
PointCloud2 用 `data.size()`），队列维护累计字节数，超预算从队首弹到达标。
条数/年龄/字节三个约束并存，任一触顶即淘汰。

**落点**：`databuffer.hpp`、`datacache_node.cpp:createCoreComponents`、config.txt。

**验收**：单测——喂入大帧使字节数先于条数触顶，验证淘汰与时间范围查询一致。

### 4.4 吞吐基准与防回归

**方案**：新增 `tools/bench_storage.cpp`（非安装目标，仅测试用）：合成
1080p 图像 + 2MB 点云按 30fps/10Hz 生成 Job 直喂 RawStorageWorker，统计稳态
处理速率与队列增长。CI 的 nightly job 跑一次，速率低于
"输入速率 × 1.2"即失败。**先用数据证明单线程是否够用，再决定是否做 4.5**。

**验收**：nightly 出现速率报告；人为劣化（如 compression_level 调到 19）能触发失败。

### 4.5 存储线程池（视 4.4 结果决定）

**方案**：仅当基准显示单线程不足时实施。`RawStorageWorker` 从单线程改为
N 个 worker（`storage_workers` 配置），**按目录哈希分片**保证同一事件目录的
Job 严格串行（manifest 追加写的顺序依赖），不同事件目录并行。

**权衡**：分片使"全局 FIFO"变为"每目录 FIFO"，预约/满员逻辑不变；
实现复杂度中等，故置于基准证据之后。

---

## 5. 可观测性（M2）

### 5.1 指标发布

**方案**：新增轻量 `MetricsRegistry`（原子计数器 + 1Hz wall_timer 序列化为
`diagnostic_msgs/DiagnosticArray` 发到 `/diagnostics`，直接被
diagnostics_aggregator 生态消费；不引第三方依赖）：

| 指标 | 来源（需补的访问器） |
|---|---|
| 各传感器缓存占用条数 | DataBuffer::occupancy() |
| 同步丢弃速率/累计 | Synchronizer 的 droppedImages_/droppedClouds_ 加 getter |
| 存储队列深度/满员丢弃次数 | RawStorageWorker::queueDepth()/drops() |
| 活跃捕获任务数 | EventMonitor activeCaptures_.size() |
| 磁盘剩余/records 占用 | DiskSpaceManager 上次探测值缓存并暴露 |
| 上传积压/失败/重试次数 | UploadWorker 待传目录数与 pending_ 统计 |
| 各传感器 stale 状态 | SensorWatchdog::describeStatus()（已有） |

**验收**：运行中 `ros2 topic echo /diagnostics` 能看到上述键；
队列打满时对应计数增长。

### 5.2 每事件元数据与审计

**方案**：

- 受理时在事件目录写 `event.json`：事件名、触发墙钟/传感器时间、pre/post 配置
  快照、软件版本（CMake 注入 `-DDATACACHE_VERSION=` git describe）、看门狗快照、
  合并/冷却历史（配合 2.4）。
- 事件级审计行（accepted/rejected + 原因）append 到 `records/events.jsonl`，
  供离线对账"触发次数 vs 落盘目录数"。

**验收**：触发一次后 `event.json` 存在且字段完整；`events.jsonl` 行数与
触发次数一致。

---

## 6. 生态互通：MCAP 导出（M3）

**方案**：`record_reader` 新增 `--export-mcast out.mcap`：用 `rosbag2_cpp::Writer`
（Jazzy 默认存储即 MCAP）把目录内记录按原始 `header.stamp` 回写为标准 bag，
Foxglove/PlotJuggler/rosbag2 工具链直接打开。**只做导出器，不改在线写入格式**——
现有 CDR+zstd 作为归档源格式保持稳定，风险最小。

**落点**：`record_reader.cpp` 新子命令；CMake 链接 `rosbag2_cpp`。

**验收**：导出的 mcap 能被 `ros2 bag info` 读取，消息数与 manifest 行数一致。

---

## 7. 扩展性（M3）

### 7.1 事件注册配置化 + 多来源触发

**方案**：

- DataCacheNode 启动时扫描配置中形如 `event_<name>_pre_time` 的键自动注册事件
  （正则 `^event_(.+)_pre_time$`），`enable_<name>_event` 缺省 true。代码中的
  collision/hard_brake 注册逻辑删除，改为纯配置驱动。新增事件类型零代码改动。
- 新增可选订阅 `/event_signal`（std_msgs/String），消息即事件名，映射到
  `triggerEvent`——CAN 网关、AD 栈软触发、脚本注入统一走这个入口；
  服务调用 `/request_trigger` 保持不变。

**验收**：配置里加 `event_lane_departure_pre_time=3` 后，
`ros2 topic pub /event_signal std_msgs/String "data: lane_departure"` 可触发录制。

### 7.2 通用传感器通道（不重构同步器）

**方案**：`SensorDataVariant` 增加第三种 `GenericData{sensorId, typeUrl,
serialized}`：节点订阅配置声明的额外话题（`extra_topics=radar:/radar_points,
imu:/imu`），消息以 CDR 原始字节直接入缓存/落盘（manifest 的 sensor 列写
sensorId，encoding 加 `raw_cdr`），**不参与同步配对**。雷达/IMU 数据由此可录制，
而同步器保持两路核心传感器的设计不变。`record_reader --verify` 对未知 sensor
按"可解压 + 字节非空"降级校验。

**权衡**：绕开 variant 强类型会失去反序列化校验，故仅对扩展传感器启用；
核心 camera/lidar 路径完全不动。

**验收**：配置雷达话题后触发事件，目录含 `radar_*.bin.zst`，verify 通过。

### 7.3 时钟偏移检测（补看门狗盲区）

**方案**：SensorWatchdog 每传感器同时维护"传感器时间速率 vs steady_clock 速率"：
`rate = Δheader.stamp / Δsteady`，滑动窗口均值偏离 1 超过
`clock_skew_tolerance_ppm`（默认 5000ppm，即 0.5%）时通过现有 Transition 机制
告警 `clock-skew`。活的但漂移的传感器不再静默切错窗口。

**验收**：单测用 1.1x 速率的假时间戳流注入，窗口内触发 skew 告警。

---

## 8. 测试深度（穿插于 M0–M3）

| 项 | 方案 | 里程碑 |
|---|---|---|
| 崩溃一致性注入 | 集成脚本：循环触发事件中途 `kill -9` 节点，断言 (a) 无残留 `.tmp` (b) manifest 中已列出的文件全部可校验或行被记为 torn（不允许"列出但缺失"）(c) 重启后对账把残留目录转为 `.failed` | M1 |
| launch 测试 | 用 `launch_testing` 起 datacache+lidar_sim+合成相机，触发并断言目录与 `.complete` 出现（把现 smoke_test 逻辑搬进 pytest，进 CI） | M2 |
| 覆盖率门禁 | CI 加 gcovr 报告与阈值（起步 60% 行覆盖，只升不降） | M2 |
| 相机硬件抽象 | camera_node 增加 `source:=synthetic` 参数（把 test_camera_publisher 的移动图案逻辑移植为 C++ 源），CI 无设备跑全链路 | M2 |
| 性能回归 | 见 4.4 nightly 基准 | M2 |
| LifecycleNode 迁移 | datacache_node 改 `rclcpp_lifecycle::LifecycleNode`，activate 前不订阅、shutdown 先停 worker 再销毁订阅；launch 用 lifecycle 事件断言状态序 | M3 |

---

## 9. 各方案对应的"已知限制"销账

| README 已知限制 | 由哪个方案解决 |
|---|---|
| 自定义 RPC 需生产安全与跨网段方案 | 3.1（DDS Security/网关）+ 4.2（断点协议 v2） |
| post 批入队失败无标记、不回传 | 2.1（`.failed`）+ 2.2（对账） |
| 视频无 h264/h265 编码 | 不解决（维持逐帧），MCAP 导出（6）改善生态侧 |
| `.upload_failed` 终态无重试 | 2.3（周期重排 + CLI） |
| 相机依赖 GUI/USB 设备 | 2.5（重试）+ 8（synthetic 源进 CI） |

---

## 10. 实施顺序（里程碑）

```
M0 工程地基（约 2–3 天）
   CI(build/test/format/sanitizer) → LICENSE/CHANGELOG → 配置键校验 → 绝对路径
M1 静默丢失清零（约 1–2 周）
   .pending/.failed 标记 → 启动对账 → record_admin CLI → 上传重排队
   → 事件冷却/合并 → camera 重试 → 崩溃注入测试
   （依赖 M0 的 CI 承接测试）
M2 安全与资源（约 1–2 周）
   DDS Security/网关 → manifest.sha256 → RPC 断点续传 → 字节预算 → 基准 nightly
   → /diagnostics 指标 → event.json 审计 → launch 测试 + 覆盖率门禁
   （依赖 M1 的标记扩展；4.5 线程池视 4.4 数据决定）
M3 生态与扩展（约 3–4 周，可拆散并行）
   MCAP 导出 → 事件注册配置化 + /event_signal → 通用传感器通道
   → 时钟偏移检测 → LifecycleNode
```

依赖关系只有三条硬边：M1 依赖 M0（CI 承接新测试）；M2 的清单哈希依赖 M1 的
`.complete` 语义扩展；M3 的通用传感器通道建议在 M2 的指标上线后做（便于观察
扩展传感器的丢弃率）。其余可并行或调整顺序。

---

## 11. 与差距分析的对照

| 差距分析条目 | 本方案章节 |
|---|---|
| 无 CI / LICENSE 占位 / 无版本机制 / 无 lint 与 sanitizer | 1.1–1.3 |
| 孤儿目录、受理≠落盘、`.upload_failed` 终态、事件风暴、camera_node 退出 | 2.1–2.5 |
| DDS RPC 未启用安全、数据无签名、配置静默默认 | 3.1–3.2、1.3 |
| 无 metrics、无事件审计 | 5.1–5.2 |
| 自定义格式不入生态、RPC 无断点续传 | 6、4.1–4.2 |
| 内存按条数、单线程吞吐未验证 | 4.3–4.5 |
| 事件硬编码、两传感器硬约束、时钟偏移盲区、相对路径 | 7.1–7.3、1.3 |
| 崩溃一致性/launch/覆盖率测试缺失 | 8 |
