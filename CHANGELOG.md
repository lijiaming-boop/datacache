# Changelog

本项目的全部显著变更记录于此。格式遵循 [Keep a Changelog](https://keepachangelog.com/zh-CN/1.1.0/)，
版本号遵循 [SemVer](https://semver.org/lang/zh-CN/)。

## [Unreleased]

### Added

- CI：GitHub Actions 三条流水线（编译+单测、clang-format 检查、ASan/UBSan 单测）。
- 许可证：仓库根目录补 MIT `LICENSE` 文件（package.xml 早已声明）。
- 配置校验：未知配置键启动时告警；`config_strict=true` 时未知键或类型不合法的值
  直接拒绝启动，消除"拼写错误静默用默认值"。
- `record_directory` 启动时解析为绝对路径并打印日志，数据不再散落于不同启动目录。
- CMake 选项 `DATACACHE_SANITIZERS=ON` 以 AddressSanitizer/UndefinedBehaviorSanitizer
  构建（CI sanitizer 任务使用）。

## [1.0.0] - 2026-08-23

首个完整功能版本：事件触发的传感器数据缓存与回流闭环。

### Added

- 事件触发录制管线：DataBuffer 环形缓存（条数+年龄+水位淘汰）、
  ApproximateSynchronizer 近似时间配对、PairIndex 同步账本、
  EventMonitor pre/post 窗口切分（传感器时间域 + 墙钟兜底 + 预约槽位）。
- 落盘：RawStorageWorker 后台线程，CDR 序列化 + zstd 压缩（XXH64 校验和）、
  OpenCV/PCL 格式转换、原子写（.tmp + fsync + rename + 目录同步）、
  `.complete` 完成标记。
- 磁盘管理：DiskSpaceManager 写前检查 + 天数/容量双保留策略。
- 回传：UploadWorker 扫描 `.complete` 目录，HTTP multipart 上传，
  指数退避重试与 `.uploaded` / `.upload_failed` 标记。
- 回读：record_io 共享库与 record_reader CLI（列表 / --verify 完整性校验 /
  --export 导出 / --sensor / --limit）。
- 可靠性：SensorWatchdog 接收时间看门狗、sync_required_for_recording 严格模式、
  O(1) 摊还热路径（分队列缓存、front 贪心配对、deque 账本）。
- 测试：8 个 gtest 套件 46 用例、端到端集成测试、smoke 脚本、mock 接收端。

[Unreleased]: https://github.com/lijiaming-boop/datacache/compare/v1.0.0...HEAD
[1.0.0]: https://github.com/lijiaming-boop/datacache/releases/tag/v1.0.0
