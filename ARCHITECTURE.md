# DataCache 架构与设计迭代文档

本文档面向维护者，回答两个问题：**系统现在由哪些模块组成、各自怎么运作**（第 2–3 章），
以及**它是如何一步步演变成现在的形态、每一步迭代的动机是什么**（第 4 章）。
用户侧的构建/运行/配置说明见 [README.md](README.md)。

---

## 1. 系统概览

DataCache 是一个 ROS2 事件触发的车载数据回流系统：相机与激光雷达数据持续流入内存环形缓存，
碰撞/急刹等事件发生时，把事件前后窗口的数据压缩落盘，后台转换为分析友好格式，
并将完整的事件目录回传到远端接收端，配套回读工具做完整性校验与导出，形成
"采集 → 缓存 → 同步 → 落盘 → 转换 → 回传 → 回读" 的完整闭环。

```
camera_node ──/image_raw──▶  ┌────────────────────────────────────┐
                             │ DataCacheNode                      │
lidar_sim_node ─/point_cloud▶│  ├ DataBuffer        (环形缓存)     │
                             │  ├ ApproximateSynchronizer (配对)   │──▶ PairIndex (账本)
event_trigger_node           │  ├ SensorWatchdog    (新鲜度监控)   │
  └─/trigger_event 服务─────▶│  ├ EventMonitor      (窗口切分/调度)│
                             │  │   ├ DiskSpaceManager (磁盘策略)  │
                             │  │   └ RawStorageWorker (落盘线程)   │
                             │  │        … 写 .complete …          │
                             │  └ UploadWorker      (回传线程)      │──HTTP──▶ 接收端
                             └────────────────────────────────────┘
                                        record_reader (离线回读/校验/导出)
```

### 1.1 进程与线程模型

| 可执行文件 | 职责 | 线程 |
|---|---|---|
| `datacache_node` | 核心节点，持有全部主要模块 | 执行器线程（订阅/服务/定时器回调）+ RawStorageWorker 后台线程 + UploadWorker 后台线程 |
| `camera_node` | USB 相机采集，发布 `/image_raw` | 执行器线程 |
| `lidar_sim_node` | 回放 PCD 文件模拟雷达，发布 `/point_cloud` | 执行器线程 |
| `event_trigger_node` | 触发代理：`/request_trigger` → `/trigger_event`，可定时自动触发 | 执行器线程 |
| `record_reader` | 离线 CLI：列出/校验/导出事件目录 | 单线程 |

关键点：**订阅回调线程上只做入队和入缓存**，序列化、zstd 压缩、JPEG/PCD 转换、
磁盘 IO、HTTP 上传全部发生在专用后台线程，回调延迟与数据规模解耦。

### 1.2 三个时钟域

系统刻意区分了三种时间，混用会导致窗口切错、清理误删或看门狗失明：

| 时钟 | 用途 | 使用者 |
|---|---|---|
| `header.stamp`（传感器时间域） | 缓存条目、配对记录、事件窗口边界的统一键 | DataBuffer、PairIndex、EventMonitor 窗口切分 |
| `steady_clock`（单调墙钟） | 接收新鲜度、重试退避、post 窗口兜底截止、清理节流 | SensorWatchdog、UploadWorker、EventMonitor wallDeadline、DiskSpaceManager |
| `system_clock`（系统墙钟） | 事件目录命名 `<event>_<纳秒>`、保留天数计算 | EventMonitor 目录命名、DiskSpaceManager |

事件窗口边界用"缓存内最新 `header.stamp`"而不是节点时钟计算，避免节点时钟与
传感器时钟偏移导致切片错位；目录命名仍用墙钟保证唯一性（两个事件可能共享同一帧
最新传感器数据）。

---

## 2. 模块运作方式

### 2.1 数据模型 — `include/data.hpp`

`CameraData` / `LidarData` 各自带 `rclcpp::Time`（取自 `header.stamp`）和消息共享指针，
用 `std::variant` 包装成 `SensorData` + `SensorType` 枚举。下游模块（DataBuffer、
EventMonitor、RawStorageWorker）通过同一个 `timestampOf(SensorData)` 访问器取时间戳，
保证"传感器时间域"的语义只在一处定义。

### 2.2 ConfigManager — `include/config_manager.hpp` / `src/config_manager.cpp`

极简的 `key=value` 文本解析（`#` 注释、两侧 trim），`getIntConfig` / `getBoolConfig`
带类型化默认值，解析失败回落默认值并告警，**保证配置文件残缺时系统仍以安全默认值启动**。

事件级配置在 EventMonitor 中实现三级回退（`getEventIntConfig`，event_monitor.hpp:241）：

```
event_<name>_<suffix>  →  event_<suffix>  →  代码默认值
例: event_collision_pre_time → event_pre_time → 5
```

这使得"每个事件可以有专属的窗口长度和传感器选择，不配则用全局值"。

### 2.3 DataBuffer — `include/databuffer.hpp`

按条数 + 年龄双约束的环形缓存，**每类传感器一条独立 `std::deque`**。

- **条数淘汰**：每队列超过 `buffer_size` 即队首弹出。
- **年龄淘汰**：维护全局水位 `latestTimestamp_`（所有传感器中最新时间戳），
  每次插入后把各队列队首 `< 水位 - maxAge` 的数据连续弹出。
  到达序 ≈ 时间戳序，因此过期数据在队首连续分布，队首弹出即可覆盖。
- **晚到拒收**：落后水位超过 `maxAge` 的乱序帧直接不入队——水位推进时它必然被淘汰，
  入队只是浪费。
- `latestSensorTimestamp()` 把"缓存视角的现在"暴露给 EventMonitor，作为事件时间锚点。

每条消息的插入成本是摊还 O(1)，与缓存规模无关（这是最后一轮性能迭代的产物，见 4.7 节）。

### 2.4 ApproximateSynchronizer — `include/approximate_synchronizer.hpp`

相机/雷达按时间戳近似配对，输出 matched 对与丢弃记录。

- **front 贪心配对**（`tryMatchLocked`，approximate_synchronizer.hpp:119）：两侧队列按
  时间戳有序，比较两个队首：差值 ≤ 容差则配对弹出；否则丢掉**更老的一侧**——
  由于有序性，老消息与未来任何对侧消息的差值只会更大，永不可能配对。
  单次到达摊还 O(1)，取代了最初的 O(N·M) 全扫描。
- **队列上限**：任一侧超过 `sync_queue_size` 丢队首并计数。
- **锁外派发**：匹配/丢弃结果先积累在栈上的 `PendingResults`，释放锁后再回调。
  回调会写 PairIndex 和日志 IO，绝不能在同步器互斥量内执行。
- **`flushUnmatched()`**：事件边界显式清算——还在等对侧的悬挂消息记为 single-sided，
  但消息本体仍在 DataBuffer 中（清账不清数据）。

### 2.5 PairIndex — `include/pair_index.hpp`

同步账本：每个 matched 对或 single-sided 丢弃都是一条 `PairRecord`（pairId 递增、
双侧时间戳、差值、状态、原因）。用途有两个：落盘为 `pairs.csv` 供离线分析同步质量；
`getDataWithinTimeRange` 供 EventMonitor 在事件窗口内查询配对情况
（`sync_required_for_recording` 严格模式据此拒绝事件）。

账本上限 10 万条，超出从 deque 队首裁剪——每条 O(1)。
（最初是 `vector` 前端 erase，上限处每条记录引发 ~10MB 整段搬移。）

### 2.6 EventMonitor — `include/event_monitor.hpp`

事件编排核心：注册事件、切窗口、调度 post 窗口、管理存储队列资源。

**触发主流程**（`recordDataAroundEvent`，event_monitor.hpp:74）：

1. `flushPendingPairs_()`：先清算同步器悬挂消息，保证账本完整。
2. 解析 pre/post 秒数（事件级 → 全局级 → 默认值三级回退）。
3. **事件时间锚定在传感器时间域**：`eventTime = dataBuffer_->latestSensorTimestamp()`，
   无传感器数据时退化为墙钟并告警。窗口 = `[eventTime - pre, eventTime]` + post 段。
4. `sync_required_for_recording` 开启时，检查 pre 窗口内是否存在 matched 配对，
   无则拒绝该事件（返回 false，触发方收到明确失败）。
5. `postSeconds = 0`：pre 窗口数据一次性入队，`finalJob=true`，流程结束。
6. 有 post 窗口：先 `storageWorker_->reserve()` **预约一个存储槽位**（防止 post 数据
   到期时队列被占满），再把 `EventCaptureTask` 放进 `activeCaptures_`（受
   `max_active_event_captures` 上限约束），最后入队 pre 批数据。pre 批入队失败则
   回滚：删除任务、释放预约、拒绝事件。

**post 窗口收割**（`processExpiredCaptures`，event_monitor.hpp:161）：
50ms 周期定时器扫描 `activeCaptures_`，任务到期条件满足即收割：

- 正常条件：最新传感器时间戳 ≥ `endTime`（传感器时间域）。
- 兜底条件：墙钟 ≥ `wallDeadline = 触发墙钟 + post + sensor_stall_grace_ms`——
  传感器时间戳停滞（设备挂了）也不会把捕获任务永久钉死在内存里。

收割时拉取 `[eventTime, endTime]` 的数据和配对，**剔除 eventTime 边界上的记录**
（pre 批已含该闭区间边界帧，不剔除会写两次：同名文件被覆盖、manifest 出现重复行），
以 `reserved=true, finalJob=true` 入队。

### 2.7 RawStorageWorker — `include/raw_storage_worker.hpp`

单后台线程消费 `jobs_` 队列（`condition_variable` 唤醒），每个 Job 携带一批
SensorData、配对记录和当次生效的全部配置快照（压缩/转换开关与参数随 Job 走，
避免 Worker 反查配置）。写入流程（`writeJob`，raw_storage_worker.hpp:295）：

1. **写前磁盘检查**：`diskManager_->prepareForWrite()`，不足则整批丢弃并报错。
2. 建目录、以追加模式打开 `manifest.csv`（首行写表头）。
3. 每条记录：
   - CDR 序列化（`rclcpp::Serialization`）得到原始字节；
   - zstd 压缩（**复用 `ZSTD_CCtx`**、**帧尾开启 XXH64 校验和**——默认
     `ZSTD_compress` 不写校验和，静默位腐只有解压时才能发现）；
   - `.tmp` 写入 → `fsync` → `rename` → `fsync` 父目录（`writeAtomically`，
     raw_storage_worker.hpp:172）。崩溃不留半文件；掉电时数据块不会晚于目录项可见；
   - 压缩失败回落写未压缩 `.bin`，manifest 的 `encoding` 列如实记录 `zstd`/`raw`；
   - 可选后台转换：OpenCV 编码 jpg（bgr8/rgb8/mono8/bgra8/rgba8）、PCL 写
     binary-compressed pcd，同样走临时文件 + rename；
   - 追加 manifest 行。
4. `pairs.csv` 同样以追加模式写入本批配对。
5. **`finalJob` 的顺序保证**：manifest/pairs 先 `flush()` 越过用户态缓冲、再 `fsync`
   越过页缓存，**然后**才原子写入 `.complete` 标记（内含 records 计数）。若标记先于
   清单落盘，进程在窗口内崩溃会留下"标记完整但 manifest 尾行缺失"的目录，上传器
   会把残缺目录当完整数据发走。
6. 最后 `enforceRetention()` 做节流后的保留清理。

**队列与预约**：`max_pending_storage_jobs` 上限；普通入队检查
`jobs_ + reservedJobs_ >= 上限`；`reserve()`/`releaseReservation()` 维护预约计数，
EventMonitor 用它保证 post 批到期时必有槽位（预约本身也计入满员判断，
普通数据无法把预约槽位挤掉）。

### 2.8 DiskSpaceManager — `include/disk_space_manager.hpp`

只被 RawStorageWorker 的后台线程调用，因此**全类无锁**。

- `prepareForWrite()`：剩余空间 ≥ `disk_min_free_mb` 放行；不足则强制清理后复查，
  仍不足返回 false（该批数据丢弃，录制失败显式上报而非写坏盘）。
- `enforceRetention()`：天数策略（目录时间戳 < now - `retention_days` 删除）+
  容量策略（records 总量超 `retention_max_capacity_mb` 从最旧删起），按
  `disk_cleanup_interval_seconds` 节流。
- **只认 `<event>_<纳秒时间戳>` 命名的目录**（`parseEventTimestamp`：取最后一个 `_`
  之后的纯数字段、至少 9 位，事件名本身可含 `_` 如 hard_brake）。命名不匹配的
  目录一律不动，避免误删人工放入的数据。
- 目录时间戳来自 system_clock 墙钟，因此年龄计算也用 system_clock。

### 2.9 SensorWatchdog — `include/sensor_watchdog.hpp`

接收时间（steady_clock）新鲜度监控，**刻意不看 `header.stamp`**：设备时钟冻结或跳变
时 header.stamp 表现"正常"，只有到达时间能暴露真实断流。

- 注册即开始计时——从未发布过数据的传感器超时后同样会被标记。
- `poll()` 由节点 wall_timer 周期驱动，**状态翻转才触发回调**（stale→fresh、
  fresh→stale 各一次），不是每次轮询都刷日志。
- `describeStatus()` 生成 `"camera: ok, lidar: STALE 2.3s"` 快照，
  附加到触发服务响应里，触发方立刻知道这次录的数据有多完整。

### 2.10 UploadWorker — `include/upload_worker.hpp`

独立后台线程，**通过文件系统标记与存储模块协作，互相不持有引用**：

```
扫描: record_root 下带 .complete、且无 .uploaded / .upload_failed 的目录
上传: 递归收集普通文件(排除点开头的标记与 .tmp), libcurl multipart POST 整目录
成功: 写 .uploaded (记录 url 与文件数)
失败: 指数退避重试 backoff * 2^(n-1) 封顶 64 倍; 超过 max_retries 写 .upload_failed(终态)
```

实现细节：上传在锁外执行（`stop()` 不必等网络超时）；禁用 `Expect: 100-continue`
（小型接收端无需续传握手）；`CURLOPT_LOW_SPEED_LIMIT/TIME` 兜底防慢速挂死；
本地数据永不删除，磁盘回收完全交给 DiskSpaceManager——**回传与清理职责分离**。

### 2.11 record_io 与 record_reader — 回读闭环

`include/record_io.hpp` 是回读共享库，**CLI 工具与单元测试共用同一份实现**，
从结构上保证"写得出就读得回"：

- CSV 解析：撕裂/畸形行记入 `problems` 上报而非抛异常（`std::stoll` 对空串/垃圾
  会抛）；整数解析拒绝残留垃圾与溢出。
- zstd 解压：帧内容大小已知时一次性解压，未知时流式兜底，可检测截断帧；
  XXH64 校验和在解压时自动验证。
- `verifyEventDirectory()` 逐条校验：文件存在 → 可解压 → CDR 反序列化 →
  消息字段自洽（图像 `step*height ≤ data.size()`、点云 `width*height*point_step ≤
  data.size()`），撕裂的 CSV 行本身计为失败。`record_reader --verify` 损坏返回
  退出码 2，可直接接入脚本判断。

### 2.12 外围节点

- **camera_node**：OpenCV `VideoCapture` 定时采帧，cv_bridge 转 `bgr8` Image。
  依赖真实 USB 设备，无相机时用 `test/test_camera_publisher.py`（30fps 合成图案）替代。
- **lidar_sim_node**：启动时加载一个 PCD 到内存，定时以当前时间戳重发
  `PointCloud2`。
- **event_trigger_node**：对外提供 `/request_trigger`（转发给 datacache_node 的
  `/trigger_event`）与可选定时自动触发。触发请求走 `async_send_request` + 完成回调——
  早期版本在回调里同步 `spin_until_future_complete`，节点已在执行器里 spin，
  重入直接崩溃（见 4.1 节）。

### 2.13 工具与测试

- `tools/mock_server.py`：纯标准库 HTTP 接收端，打印收到的文件清单与 md5，用于
  回传闭环验证。
- `tools/smoke_test.sh`：端到端冒烟——起 mock 端与雷达仿真、触发事件、跑
  `record_reader --verify/--export`、验证 `.uploaded` 与接收端落盘。
- `tools/vm_exec.py`：向验证用 Ubuntu VM 传文件/执行命令（凭据走环境变量）。
- 单元测试 8 个 gtest 套件 46 用例：ConfigManager、DataBuffer、PairIndex、
  ApproximateSynchronizer、record_io、UploadWorker（进程内起 HTTP server）、
  EventMonitor（窗口切分/双时钟到期/预约回滚/严格模式/并发上限）、DiskSpaceManager。
- 集成测试 `test/run_integration_test.sh` + `validate_records.py`：全链路起节点、
  触发双事件、对产物做结构化断言。

---

## 3. 两条关键数据流走查

### 3.1 一条消息的热路径（每次到达都执行）

```
订阅回调 (SensorDataQoS)
  ├─ watchdog_->noteData()          # O(1), steady_clock 打点
  ├─ dataBuffer_->addData()         # O(1) 摊还: 入队 + 条数/水位淘汰
  └─ synchronizer_->addImage/Cloud()  # O(1) 摊还: 锁内 front 配对/丢弃
                                        锁外回调 ──▶ pairIndex_->addMatched/addSingle  # O(1) 摊还
```

全程无序列化、无压缩、无磁盘 IO、无内存大搬移，处理成本不随缓存/账本规模增长。

### 3.2 一个事件的生命周期

```
ros2 service call /request_trigger
  → event_trigger_node (异步转发)
  → datacache_node /trigger_event
  → EventMonitor::recordDataAroundEvent
       flush 同步器悬挂消息
       eventTime = 最新 header.stamp (传感器域)
       [严格模式] pre 窗口无 matched 对 → 拒绝, 响应附看门狗状态
       reserve() 预约 post 槽位; 建 EventCaptureTask (wallDeadline 兜底)
       pre 批入队 (普通优先级)
  → RawStorageWorker 线程: CDR → zstd(XXH64) → .tmp+fsync+rename
       manifest.csv / pairs.csv 追加
  … post 秒后, 调度定时器发现 sensorNow ≥ endTime (或墙钟兜底到期)
  → post 批入队 (reserved=true, finalJob=true, 剔除 eventTime 边界重复帧)
  → Worker: 清单 flush+fsync → 写 .complete
  → UploadWorker 扫描周期发现 → multipart POST → .uploaded (或退避重试→.upload_failed)
```

事件目录内的标记文件构成一个**文件系统状态机**，是跨模块、跨进程协作的唯一接口：

| 标记 | 语义 | 生产者 → 消费者 |
|---|---|---|
| `.tmp` | 写入中，任何读者都应忽略 | RawStorageWorker → 所有读者 |
| `.complete` | 目录内容完整，可回传 | RawStorageWorker → UploadWorker |
| `.uploaded` | 已成功回传（含 url/文件数） | UploadWorker → 运维 |
| `.upload_failed` | 重试耗尽，终态，需人工介入 | UploadWorker → 运维 |

---

## 4. 设计迭代思路

提交历史呈现清晰的分阶段演进：每轮迭代都由一个明确的痛点驱动，且每轮都留下
验证手段（测试/工具），这是本项目最值得复用的方法论。以下按时间线梳理。

### 4.0 阶段零：跑通最小闭环（a47c8de，08-20）

初始提交包含四个节点 + DataBuffer + 事件触发录制的基本骨架。随后 7 个提交全是
构建与运行期修复：

- CMake/ament 安装规则不全导致节点不可执行；
- VTK 经 PCL 传递依赖时 `VTK::mpi` target 缺失（需先 `find_package(MPI COMPONENTS C CXX)`
  并启用 C 语言）；
- 运行目录 ≠ 安装目录，config/pcd 路径改从 package share 解析；
- 空触发请求导致崩溃，改为回落默认 collision 事件；
- **执行器重入崩溃**：服务回调里同步 `spin_until_future_complete`，而节点已在
  执行器中 spin。修复为 `async_send_request` + 完成回调（97e9c45）。
  这一教训直接塑造了 event_trigger_node 的最终形态（2.12 节）。

> 迭代启示：ROS2 + PCL + OpenCV 的交叉构建环境本身就是一个集成难题，
> "先让它在真机/VM 上完整跑起来"是后续一切迭代的地基。

### 4.1 阶段一：异步化存储管线（ba627ef，PR #1）

初版事件录制在回调线程上直接写磁盘。引入 **RawStorageWorker 后台线程 + 有界
Job 队列**：订阅/服务回调只做数据拷贝入队，序列化与 IO 移出热路径，同时加入
zstd 压缩。这是"**热路径最小化**"原则的第一次落地，后续所有模块设计都延续
这一分工（回调入队、后台消费）。

### 4.2 阶段二：同步账本与格式转换（aea9599 + 64a4f45，PR #2）

车载分析需要知道"哪些帧是相机/雷达对齐的"。加入：

- **ApproximateSynchronizer**（±tolerance 时间配对）与 **PairIndex** 账本；
- 事件级配置覆盖（每事件独立窗口与传感器选择）；
- OpenCV/PCL 后台格式转换，产出分析友好副本。

Code review 发现两个典型缺陷并在合入前修复（64a4f45）：

- `tryMatch` 在 `pop_front()` 之后调用 `front()` 读"被丢弃消息的时间戳"，
  实际读到的是下一条消息（队刚空时是 UB）——**先取值再弹出**；
- manifest 的 `.zst` 后缀在写结果确认之前就拼好，压缩失败走 raw 回退时
  清单指向不存在的文件——**写成功后才提交名字**。

> 迭代启示：review 对"指针/迭代器失效后使用"和"错误路径与主路径的状态提交顺序"
> 这两类问题最有效，恰好都是并发 IO 代码的高发区。

### 4.3 阶段三：端到端集成测试（f771505 + 7702dd6，PR #3）

阶段二的合入曾因两处括号缺失直接打破编译（`.string()` 作用于 `std::string`、
成员初始化列表括号不闭合），说明**缺乏自动编译/测试门禁**。本阶段补齐：

- 合成相机发布器（无真实相机也能全链路跑）；
- 集成测试脚本 + `validate_records.py` 结构化断言（manifest/zstd/转换产物/
  pairs 容差/事件级传感器选择）；
- VM 上验证：1013 断言全过。

用于定位该编译错误的一次性脚本（括号平衡扫描等）在后续 a66d11e 中删除——
**调试脚本是耗材，验证脚本是资产**，两者分开对待。

### 4.4 阶段四：车载可靠性三件套（65d239a）

把"演示系统"推向"可部署系统"，针对三个真实失效模式：

1. **时钟偏移** → 事件窗口全面改用传感器时间域（锚定最新 header.stamp），
   post 窗口另设墙钟兜底截止（`sensor_stall_grace_ms`），传感器停滞不再钉死任务。
2. **磁盘耗尽** → DiskSpaceManager：写前检查 + 天数/容量双保留 + 从最旧回收。
3. **传感器断流** → SensorWatchdog（接收时间域），状态附到触发响应；
   加上 `sync_required_for_recording` 严格模式：pre 窗口无配对即拒绝录制。

同时把 datacache.hpp 巨头文件拆为 hpp/cpp 单元——为下一轮可测试性铺路
（EventMonitor 的测试需要链接最小依赖集）。

### 4.5 阶段五：热路径性能（a587fac）

VM 上压测发现两个随规模恶化的点，均以数据结构换算法：

| 问题 | 原实现 | 新实现 | 收益（-O2 实测） |
|---|---|---|---|
| 同步配对 | 每次到达 O(N·M) 双队列全扫描 | front 贪心 + 单调性剪枝 | 摊还 O(1) |
| PairIndex 裁剪 | `vector` 前端 erase，10 万上限处每条 ~10MB 搬移 | `deque` 队首弹出 | ~980x |
| 回调持锁 | PairIndex/日志在同步器锁内执行 | `PendingResults` 锁外派发 | 消除锁竞争 |

DataBuffer 同期改为按传感器分队列 + 水位队首弹出（385x @ cap）。
这一轮确立了"**每条消息的处理成本有平摊上界**"的性能不变量，并在注释中写明了
维持该不变量的前提（到达序 ≈ 时间戳序），防止后人无意破坏。

### 4.6 阶段六：补全写读与回传闭环（cf053fd + 361d037）

系统此前"只写不读、只存不传"，数据落盘后无法验证也无法回流：

- **record_io / record_reader**：回读实现与 CLI（列出/--verify/--export），
  与单测共用同一库——写读对齐由结构保证而非约定保证；
- **UploadWorker + 标记状态机**：`.complete` → multipart POST → `.uploaded` /
  `.upload_failed`，指数退避封顶 64 倍；
- `mock_server.py` 让回传闭环无需外部依赖即可验证；
- 首批 6 个单测套件（30 用例）。

> 迭代启示：`事件目录` 是存储、上传、清理、回读四个模块的**唯一共享契约**，
> 把契约固化成标记文件 + manifest/pairs 格式后，各模块可以独立演进甚至独立部署。

### 4.7 阶段七：存储耐久性（a66d11e，当前 HEAD）

最后一轮针对"数据写着写着就没了/坏了"的静默失效，全部是**顺序与校验**问题：

- **fsync 顺序**：数据文件先 fsync 再 rename、rename 后同步父目录项——rename 只
  保证原子可见，不保证数据块先于目录项持久化，掉电会留下"标记完整内容为空"的文件；
- **`.complete` 后置**：必须在 manifest/pairs flush + fsync 之后写入（2.7 节）；
- **XXH64 校验和**：zstd 默认不写帧校验，静默位腐要等到下游分析失败才暴露；
- **撕裂行降级**：崩溃残留的半行 CSV 从抛异常改为记入 problems，`--verify`
  对其返回失败——校验工具自己不能是崩溃源；
- **eventTime 边界去重**：post 批剔除边界帧，消除重复文件与 manifest 重复行；
- 测试从 24 增至 46（EventMonitor 套件把 `processExpiredCaptures` 设为 public
  作为测试缝合点，DiskSpaceManager 套件从临时 harness 移植转正）。

### 4.8 贯穿各阶段的迭代原则

1. **热路径最小化**：回调线程只入队；序列化/压缩/IO/网络全部后台线程化。
2. **时钟域显式分离**：传感器域切窗口、单调钟管调度与退避、墙钟管命名与年龄，
   每处用哪个钟都有注释说明为什么。
3. **文件系统即状态机**：`.tmp/.complete/.uploaded/.upload_failed` 是跨模块契约，
   原子写 + fsync 保证状态迁移本身可信。
4. **写读闭环**：任何落盘格式必须有配套回读实现与损坏检测，且二者共享代码。
5. **故障显式建模**：队列满丢弃计数、预约槽位、看门狗、墙钟兜底、磁盘保留、
   重试终态——每种失效都有名字、有日志、有标记，而不是静默吞掉。
6. **每轮迭代留下验证资产**：单测/集成测试/冒烟脚本/mock 接收端，一次性调试
   脚本用完即删。
7. **测试缝合点前置设计**：核心调度函数 public 化、Worker 参数全量注入，
   让测试不依赖定时器被 spin 到这类时序运气。

---

## 5. 已知限制与可能的下一步

与 README「已知限制」一致，同时是下一轮迭代的自然候选：

- 上传为自定义 multipart 约定，生产接收端需按 `mock_server.py` 的约定解析；
  大目录上传峰值内存可优化为 `curl_mime_file_cb` 流式读取（代码内有注释留位）。
- post 批入队失败（队列满）只报错，事件目录无 `.complete` 也不会回传——
  可考虑失败标记 + 人工/自动重试入口。
- `.upload_failed` 是终态，无自动重试入口，需人工删除标记。
- 视频片段为逐帧图像，无 h264/h265 编码；`pointcloud_format` 仅支持 pcd。
- DataBuffer 的 `getDataWithinTimeRange` 仍是线性扫描（仅事件时调用，不在热路径）；
  若窗口查询频度上升可加有序索引。
