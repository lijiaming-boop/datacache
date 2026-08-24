#pragma once

// 上传/回传模块: 后台线程扫描 record_directory, 把带 .complete 标记(由
// RawStorageWorker 在事件目录写完后生成)的事件目录通过 ROS2 服务调用
// (datacache/srv/UploadStore, BEGIN -> FILE_CHUNK* -> END)整目录回传。
// 成功写 .uploaded, 重试耗尽写 .upload_failed。本地数据不删除, 磁盘回收
// 仍由 DiskSpaceManager 负责。
//
// 注意: 服务响应依赖调用方节点的执行器在 spin, UploadWorker 的后台线程
// 只负责发起异步请求并等待 future。

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <fstream>
#include <future>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <rclcpp/client.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>

#include "datacache/srv/upload_store.hpp"
#include "event_state.hpp"

class UploadWorker {
public:
    using UploadStore = datacache::srv::UploadStore;
    using UploadRequest = UploadStore::Request;
    using UploadResponse = UploadStore::Response;

    struct Config {
        std::string serviceName{"/upload_store"}; // 接收端服务名
        long timeoutSeconds{30};                  // 单次 RPC 调用超时
        int maxRetries{5};
        std::chrono::milliseconds scanPeriod{2000};
        std::chrono::milliseconds retryBackoff{15000}; // 首次重试等待, 之后指数翻倍
        std::size_t chunkBytes{512 * 1024};            // 单块负载上限(srv 约束 1MiB 内)
        std::chrono::seconds leaseTimeout{300};
        std::chrono::milliseconds failedRescanPeriod{1800000};
    };

    static constexpr std::size_t kMaxChunkBytes = 1024 * 1024;

    UploadWorker(std::filesystem::path recordRoot, Config config, rclcpp::Logger logger,
                 rclcpp::Node* node)
        : recordRoot_(std::move(recordRoot)), config_(std::move(config)),
          logger_(std::move(logger)),
          client_(node->create_client<UploadStore>(config_.serviceName)) {
        if (config_.chunkBytes == 0 || config_.chunkBytes > kMaxChunkBytes) {
            const auto requested = config_.chunkBytes;
            config_.chunkBytes = std::clamp(config_.chunkBytes, std::size_t{1}, kMaxChunkBytes);
            RCLCPP_WARN(logger_, "Invalid RPC chunk size %zu; clamped to %zu bytes", requested,
                        config_.chunkBytes);
        }
    }

    UploadWorker(const UploadWorker&) = delete;
    UploadWorker& operator=(const UploadWorker&) = delete;

    ~UploadWorker() { stop(); }

    void start() {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (started_ || stopping_.load()) {
                return;
            }
            started_ = true;
        }
        worker_ = std::thread(&UploadWorker::run, this);
    }

    void stop() {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            stopping_.store(true);
        }
        condition_.notify_all();
        if (worker_.joinable()) {
            worker_.join();
        }
    }

    // 接收端服务是否已在本 DDS 域内被发现(后台线程据此快速跳过本轮)
    bool serviceReady() const { return client_->service_is_ready(); }

    // 扫描可回传目录: 带 .complete 标记且尚未 .uploaded / .upload_failed
    std::vector<std::filesystem::path> findUploadCandidates() {
        std::vector<std::filesystem::path> candidates;
        std::error_code error;
        for (std::filesystem::directory_iterator it(recordRoot_, error), end; !error && it != end;
             it.increment(error)) {
            std::error_code entryError;
            if (!it->is_directory(entryError) || entryError) {
                continue;
            }
            const auto directory = it->path();
            if (leaseActive(directory)) {
                continue;
            }
            if (std::filesystem::exists(directory / ".uploaded") ||
                !std::filesystem::exists(directory / ".complete")) {
                continue;
            }
            const auto failedMarker = directory / ".upload_failed";
            if (std::filesystem::exists(failedMarker)) {
                if (config_.failedRescanPeriod.count() <= 0) {
                    continue;
                }
                std::error_code timeError;
                const auto modified = std::filesystem::last_write_time(failedMarker, timeError);
                if (timeError ||
                    decltype(modified)::clock::now() - modified < config_.failedRescanPeriod) {
                    continue;
                }
                std::error_code removeError;
                std::filesystem::remove(failedMarker, removeError);
                if (removeError) {
                    continue;
                }
            }
            candidates.push_back(directory);
        }
        return candidates;
    }

    // 单目录一次上传(不含重试)。成功后写 .uploaded 标记。
    bool uploadDirectory(const std::filesystem::path& directory) {
        if (!acquireLease(directory)) {
            RCLCPP_WARN(logger_, "Upload lease is already active for %s",
                        directory.filename().string().c_str());
            return false;
        }
        struct LeaseGuard {
            std::filesystem::path marker;
            ~LeaseGuard() {
                std::error_code error;
                std::filesystem::remove(marker, error);
            }
        } lease{directory / ".uploading"};
        if (!transferDirectory(directory)) {
            return false;
        }
        std::ostringstream content;
        content << "service=" << config_.serviceName
                << "\nfiles=" << countUploadableFiles(directory) << "\n";
        return event_state::writeAtomically(directory / ".uploaded", content.str());
    }

private:
    struct RetryState {
        int attempts{0};
        std::chrono::steady_clock::time_point nextAttempt{};
    };

    static std::int64_t systemNowNs() {
        return std::chrono::duration_cast<std::chrono::nanoseconds>(
                   std::chrono::system_clock::now().time_since_epoch())
            .count();
    }

    bool leaseActive(const std::filesystem::path& directory) {
        const auto marker = directory / ".uploading";
        std::ifstream input(marker);
        if (!input) {
            return false;
        }
        std::string key;
        std::int64_t startedAt = 0;
        if (std::getline(input, key, '=') && key == "started_at_ns" && input >> startedAt) {
            const auto age = systemNowNs() - startedAt;
            if (age >= 0 &&
                age < std::chrono::duration_cast<std::chrono::nanoseconds>(config_.leaseTimeout)
                          .count()) {
                return true;
            }
        }
        std::error_code error;
        std::filesystem::remove(marker, error);
        return false;
    }

    bool acquireLease(const std::filesystem::path& directory) {
        if (leaseActive(directory)) {
            return false;
        }
        return event_state::writeAtomically(
            directory / ".uploading", "started_at_ns=" + std::to_string(systemNowNs()) + "\n");
    }

    static bool isMarkerOrTemp(const std::filesystem::path& file) {
        const auto name = file.filename().string();
        if (name.size() > 1 && name[0] == '.') {
            return true; // .complete / .uploaded / .upload_failed
        }
        return file.extension() == ".tmp";
    }

    static std::vector<std::filesystem::path> collectFiles(const std::filesystem::path& directory,
                                                           bool* complete = nullptr) {
        std::vector<std::filesystem::path> files;
        std::error_code error;
        for (std::filesystem::recursive_directory_iterator it(directory, error), end;
             !error && it != end; it.increment(error)) {
            std::error_code entryError;
            const bool symlink = it->is_symlink(entryError);
            if (!entryError && !symlink && it->is_regular_file(entryError) && !entryError &&
                !isMarkerOrTemp(it->path())) {
                files.push_back(it->path());
            }
        }
        if (complete != nullptr) {
            *complete = !error;
        }
        std::sort(files.begin(), files.end());
        return files;
    }

    static std::size_t countUploadableFiles(const std::filesystem::path& directory) {
        return collectFiles(directory).size();
    }

    static bool fileSize(const std::filesystem::path& file, std::uintmax_t& size) {
        std::error_code error;
        size = std::filesystem::file_size(file, error);
        return !error;
    }

    static std::string transferIdFor(const std::filesystem::path& directory) {
        const auto marker = directory / ".upload_id";
        std::ifstream input(marker);
        std::string transferId;
        if (input && std::getline(input, transferId) && !transferId.empty()) {
            return transferId;
        }
        static std::atomic<std::uint64_t> nextId{0};
        transferId = std::to_string(systemNowNs()) + "-" + std::to_string(nextId.fetch_add(1));
        if (!event_state::writeAtomically(marker, transferId + "\n")) {
            return {};
        }
        return transferId;
    }

    // 发送一次 RPC 并等待响应; 服务不可用或超时返回 nullptr
    std::shared_ptr<UploadResponse> call(const std::shared_ptr<UploadRequest>& request) const {
        if (!client_->service_is_ready()) {
            RCLCPP_ERROR(logger_, "Upload service %s is not available",
                         config_.serviceName.c_str());
            return nullptr;
        }
        auto future = client_->async_send_request(request);
        const auto deadline =
            std::chrono::steady_clock::now() + std::chrono::seconds(config_.timeoutSeconds);
        while (future.wait_for(std::chrono::milliseconds(100)) != std::future_status::ready) {
            if (stopping_.load()) {
                client_->remove_pending_request(future);
                return nullptr;
            }
            if (std::chrono::steady_clock::now() >= deadline) {
                client_->remove_pending_request(future);
                RCLCPP_ERROR(logger_, "Upload RPC %s timed out after %lds",
                             config_.serviceName.c_str(), config_.timeoutSeconds);
                return nullptr;
            }
        }
        try {
            return future.get();
        } catch (const std::exception& error) {
            RCLCPP_ERROR(logger_, "Upload RPC %s failed: %s", config_.serviceName.c_str(),
                         error.what());
            return nullptr;
        }
    }

    // 整目录传输: BEGIN -> 每个文件按块 FILE_CHUNK -> END。
    // 接收端在 BEGIN 时重建暂存目录, 失败后从头重传即天然幂等。
    bool transferDirectory(const std::filesystem::path& directory) const {
        bool traversalComplete = false;
        const auto files = collectFiles(directory, &traversalComplete);
        if (!traversalComplete) {
            RCLCPP_ERROR(logger_, "Cannot traverse complete upload directory %s",
                         directory.string().c_str());
            return false;
        }
        if (files.empty()) {
            RCLCPP_WARN(logger_, "Skipping upload of %s: no uploadable files",
                        directory.filename().string().c_str());
            return false;
        }
        const auto eventName = directory.filename().string();
        const auto transferId = transferIdFor(directory);
        if (transferId.empty()) {
            RCLCPP_ERROR(logger_, "Cannot create upload transaction id for %s", eventName.c_str());
            return false;
        }

        struct FileInfo {
            std::filesystem::path path;
            std::string relative;
            std::uintmax_t size;
        };
        if (files.size() > std::numeric_limits<std::uint32_t>::max()) {
            RCLCPP_ERROR(logger_, "Upload of %s has too many files", eventName.c_str());
            return false;
        }
        std::vector<FileInfo> fileInfo;
        fileInfo.reserve(files.size());
        std::uintmax_t totalBytes = 0;
        for (const auto& file : files) {
            std::uintmax_t size = 0;
            if (!fileSize(file, size) ||
                size > std::numeric_limits<std::uint64_t>::max() - totalBytes) {
                RCLCPP_ERROR(logger_, "Cannot determine safe upload size for %s",
                             file.string().c_str());
                return false;
            }
            std::error_code relativeError;
            auto relative = std::filesystem::relative(file, directory, relativeError);
            if (relativeError || relative.empty()) {
                RCLCPP_ERROR(logger_, "Cannot make upload path relative: %s",
                             file.string().c_str());
                return false;
            }
            totalBytes += size;
            fileInfo.push_back({file, relative.generic_string(), size});
        }

        auto request = std::make_shared<UploadRequest>();
        request->event_name = eventName;
        request->transfer_id = transferId;
        request->opcode = UploadRequest::BEGIN;
        request->file_count = static_cast<std::uint32_t>(files.size());
        request->total_bytes = static_cast<std::uint64_t>(totalBytes);
        auto response = call(request);
        if (!response || !response->success) {
            RCLCPP_ERROR(logger_, "Upload of %s rejected at begin: %s", eventName.c_str(),
                         response ? response->message.c_str() : "service unavailable");
            return false;
        }

        std::uintmax_t sentBytes = 0;
        std::vector<std::uint8_t> chunk(config_.chunkBytes);
        for (const auto& file : fileInfo) {
            std::ifstream input(file.path, std::ios::binary);
            if (!input) {
                RCLCPP_ERROR(logger_, "Cannot read %s for upload", file.path.string().c_str());
                return false;
            }
            std::uintmax_t offset = 0;
            // 空文件也发一个零长度 FILE_CHUNK，令接收端能建立文件并计入文件数。
            if (file.size == 0) {
                request = std::make_shared<UploadRequest>();
                request->event_name = eventName;
                request->transfer_id = transferId;
                request->opcode = UploadRequest::FILE_CHUNK;
                request->file_path = file.relative;
                response = call(request);
                if (!response || !response->success) {
                    RCLCPP_ERROR(logger_, "Upload of empty file %s:%s failed", eventName.c_str(),
                                 file.relative.c_str());
                    return false;
                }
            }
            while (offset < file.size) {
                input.read(reinterpret_cast<char*>(chunk.data()),
                           static_cast<std::streamsize>(chunk.size()));
                const auto got = input.gcount();
                if (got <= 0) {
                    break;
                }
                request = std::make_shared<UploadRequest>();
                request->event_name = eventName;
                request->transfer_id = transferId;
                request->opcode = UploadRequest::FILE_CHUNK;
                request->file_path = file.relative;
                request->offset = static_cast<std::uint64_t>(offset);
                request->total_bytes = static_cast<std::uint64_t>(file.size);
                request->data.assign(chunk.begin(), chunk.begin() + got);
                response = call(request);
                if (!response || !response->success) {
                    RCLCPP_ERROR(logger_, "Upload of %s:%s failed at offset %llu",
                                 eventName.c_str(), file.relative.c_str(),
                                 static_cast<unsigned long long>(offset));
                    return false;
                }
                offset += static_cast<std::uintmax_t>(got);
                sentBytes += static_cast<std::uintmax_t>(got);
            }
            std::uintmax_t finalSize = 0;
            if (offset != file.size || !fileSize(file.path, finalSize) || finalSize != file.size) {
                RCLCPP_ERROR(logger_, "File %s changed size during upload",
                             file.path.string().c_str());
                return false;
            }
        }

        request = std::make_shared<UploadRequest>();
        request->event_name = eventName;
        request->transfer_id = transferId;
        request->opcode = UploadRequest::END;
        response = call(request);
        if (!response || !response->success) {
            RCLCPP_ERROR(logger_, "Upload of %s rejected at end: %s", eventName.c_str(),
                         response ? response->message.c_str() : "service unavailable");
            return false;
        }
        if (response->received_bytes != sentBytes) {
            RCLCPP_ERROR(logger_, "Upload of %s: receiver verified %llu bytes, sender sent %llu",
                         eventName.c_str(),
                         static_cast<unsigned long long>(response->received_bytes),
                         static_cast<unsigned long long>(sentBytes));
            return false;
        }
        RCLCPP_INFO(logger_, "Uploaded %s (%zu files, %llu bytes)", eventName.c_str(), files.size(),
                    static_cast<unsigned long long>(sentBytes));
        return true;
    }

    void run() {
        RCLCPP_INFO(logger_, "Upload worker started: root=%s service=%s scan=%lldms",
                    recordRoot_.string().c_str(), config_.serviceName.c_str(),
                    static_cast<long long>(config_.scanPeriod.count()));
        while (true) {
            const auto scanNow = std::chrono::steady_clock::now();
            for (const auto& directory : findUploadCandidates()) {
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    if (stopping_.load()) {
                        return;
                    }
                    const auto state = pending_.find(directory.string());
                    if (state != pending_.end() && scanNow < state->second.nextAttempt) {
                        continue;
                    }
                }

                // 上传在锁外执行, stop() 不必等待 RPC 超时
                if (uploadDirectory(directory)) {
                    std::lock_guard<std::mutex> lock(mutex_);
                    pending_.erase(directory.string());
                    continue;
                }

                std::lock_guard<std::mutex> lock(mutex_);
                if (stopping_.load()) {
                    return;
                }
                auto& state = pending_[directory.string()];
                ++state.attempts;
                if (config_.maxRetries > 0 && state.attempts >= config_.maxRetries) {
                    RCLCPP_ERROR(logger_, "Upload of %s exceeded %d retries; marked failed",
                                 directory.filename().string().c_str(), config_.maxRetries);
                    if (event_state::writeAtomically(directory / ".upload_failed",
                                                     "attempts=" + std::to_string(state.attempts) +
                                                         "\n")) {
                        pending_.erase(directory.string());
                        continue;
                    }
                    RCLCPP_ERROR(logger_, "Cannot persist upload failure marker for %s",
                                 directory.filename().string().c_str());
                }
                // 指数退避: backoff * 2^(attempts-1), 封顶 64 倍
                const auto delay = config_.retryBackoff * (1LL << std::min(state.attempts - 1, 6));
                state.nextAttempt = std::chrono::steady_clock::now() + delay;
                RCLCPP_WARN(logger_, "Upload of %s failed (%d/%d); retry in %lld ms",
                            directory.filename().string().c_str(), state.attempts,
                            config_.maxRetries, static_cast<long long>(delay.count()));
            }

            std::unique_lock<std::mutex> lock(mutex_);
            if (stopping_.load()) {
                return;
            }
            condition_.wait_for(lock, config_.scanPeriod);
        }
    }

    std::filesystem::path recordRoot_;
    Config config_;
    rclcpp::Logger logger_;
    rclcpp::Client<UploadStore>::SharedPtr client_;
    std::map<std::string, RetryState> pending_;
    std::thread worker_;
    std::mutex mutex_;
    std::condition_variable condition_;
    std::atomic<bool> stopping_{false};
    bool started_{false};
};
