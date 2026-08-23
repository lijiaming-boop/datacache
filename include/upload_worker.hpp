#pragma once

// 上传/回传模块: 后台线程扫描 record_directory, 把带 .complete 标记(由
// RawStorageWorker 在事件目录写完后生成)的事件目录通过 HTTP multipart POST
// 整目录回传到 upload_url。成功写 .uploaded, 重试耗尽写 .upload_failed。
// 本地数据不删除, 磁盘回收仍由 DiskSpaceManager 负责。

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <curl/curl.h>
#include <filesystem>
#include <fstream>
#include <map>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

class UploadWorker {
public:
    struct Config {
        std::string url;  // 接收端地址, 如 http://192.168.1.10:8080/upload
        long timeoutSeconds{30};
        int maxRetries{5};
        std::chrono::milliseconds scanPeriod{2000};
        std::chrono::milliseconds retryBackoff{15000};  // 首次重试等待, 之后指数翻倍
    };

    UploadWorker(std::filesystem::path recordRoot, Config config, rclcpp::Logger logger)
        : recordRoot_(std::move(recordRoot)), config_(std::move(config)),
          logger_(std::move(logger)) {
        curl_global_init(CURL_GLOBAL_DEFAULT);
    }

    UploadWorker(const UploadWorker&) = delete;
    UploadWorker& operator=(const UploadWorker&) = delete;

    ~UploadWorker() {
        stop();
        curl_global_cleanup();
    }

    void start() {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (started_ || stopping_) {
                return;
            }
            started_ = true;
        }
        worker_ = std::thread(&UploadWorker::run, this);
    }

    void stop() {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            stopping_ = true;
        }
        condition_.notify_all();
        if (worker_.joinable()) {
            worker_.join();
        }
    }

    // 扫描可回传目录: 带 .complete 标记且尚未 .uploaded / .upload_failed
    std::vector<std::filesystem::path> findUploadCandidates() const {
        std::vector<std::filesystem::path> candidates;
        std::error_code error;
        for (std::filesystem::directory_iterator it(recordRoot_, error), end;
             !error && it != end; it.increment(error)) {
            std::error_code entryError;
            if (!it->is_directory(entryError) || entryError) {
                continue;
            }
            const auto directory = it->path();
            if (std::filesystem::exists(directory / ".uploaded") ||
                std::filesystem::exists(directory / ".upload_failed") ||
                !std::filesystem::exists(directory / ".complete")) {
                continue;
            }
            candidates.push_back(directory);
        }
        return candidates;
    }

    // 单目录一次上传(不含重试)。成功后写 .uploaded 标记。
    bool uploadDirectory(const std::filesystem::path& directory) {
        if (!postDirectory(directory)) {
            return false;
        }
        std::ostringstream content;
        content << "url=" << config_.url << "\nfiles="
                << countUploadableFiles(directory) << "\n";
        std::ofstream output(directory / ".uploaded", std::ios::trunc);
        output << content.str();
        return static_cast<bool>(output);
    }

private:
    struct RetryState {
        int attempts{0};
        std::chrono::steady_clock::time_point nextAttempt{};
    };

    static bool isMarkerOrTemp(const std::filesystem::path& file) {
        const auto name = file.filename().string();
        if (name.size() > 1 && name[0] == '.') {
            return true;  // .complete / .uploaded / .upload_failed
        }
        return file.extension() == ".tmp";
    }

    static std::vector<std::filesystem::path> collectFiles(
        const std::filesystem::path& directory) {
        std::vector<std::filesystem::path> files;
        std::error_code error;
        for (std::filesystem::recursive_directory_iterator it(directory, error), end;
             !error && it != end; it.increment(error)) {
            std::error_code entryError;
            if (it->is_regular_file(entryError) && !entryError &&
                !isMarkerOrTemp(it->path())) {
                files.push_back(it->path());
            }
        }
        std::sort(files.begin(), files.end());
        return files;
    }

    static std::size_t countUploadableFiles(const std::filesystem::path& directory) {
        return collectFiles(directory).size();
    }

    static std::size_t writeResponse(void* data, std::size_t size, std::size_t count,
                                     std::string* response) {
        const auto bytes = size * count;
        response->append(static_cast<const char*>(data), bytes);
        return bytes;
    }

    static bool readFileBytes(const std::filesystem::path& file,
                              std::vector<std::uint8_t>& bytes) {
        std::ifstream input(file, std::ios::binary | std::ios::ate);
        if (!input.is_open()) {
            return false;
        }
        const auto size = input.tellg();
        if (size <= 0) {
            bytes.clear();
            return true;
        }
        bytes.resize(static_cast<std::size_t>(size));
        input.seekg(0);
        input.read(reinterpret_cast<char*>(bytes.data()),
                   static_cast<std::streamsize>(bytes.size()));
        return static_cast<bool>(input);
    }

    // 读取整个目录为 multipart/form-data 一次 POST 出去
    bool postDirectory(const std::filesystem::path& directory) const {
        const auto files = collectFiles(directory);
        if (files.empty()) {
            RCLCPP_WARN(logger_, "Skipping upload of %s: no uploadable files",
                        directory.filename().string().c_str());
            return false;
        }

        CURL* curl = curl_easy_init();
        if (!curl) {
            RCLCPP_ERROR(logger_, "Cannot initialize libcurl for upload");
            return false;
        }

        // curl_mime_data 本身会拷贝数据, 缓冲区保留到 perform 结束只是保守起见;
        // 大目录上传的峰值内存优化(改用 curl_mime_file_cb 流式读取)留作后续
        std::vector<std::vector<std::uint8_t>> buffers;
        buffers.reserve(files.size());
        curl_mime* mime = curl_mime_init(curl);
        for (const auto& file : files) {
            auto part = curl_mime_addpart(mime);
            curl_mime_name(part, "files");
            curl_mime_filename(
                part, std::filesystem::relative(file, directory).generic_string().c_str());
            buffers.emplace_back();
            if (!readFileBytes(file, buffers.back())) {
                RCLCPP_ERROR(logger_, "Cannot read %s for upload", file.string().c_str());
                curl_mime_free(mime);
                curl_easy_cleanup(curl);
                return false;
            }
            curl_mime_data(part, reinterpret_cast<const char*>(buffers.back().data()),
                           buffers.back().size());
        }
        const auto eventName = directory.filename().string();
        auto eventPart = curl_mime_addpart(mime);
        curl_mime_name(eventPart, "event");
        curl_mime_data(eventPart, eventName.c_str(), eventName.size());

        // 禁用 Expect: 100-continue, 小型接收端无需实现续传握手
        struct curl_slist* headers = nullptr;
        headers = curl_slist_append(headers, "Expect:");

        const auto url = config_.url + "?event=" + escapeUrl(curl, eventName);

        std::string response;
        curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
        curl_easy_setopt(curl, CURLOPT_MIMEPOST, mime);
        curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
        curl_easy_setopt(curl, CURLOPT_TIMEOUT, config_.timeoutSeconds);
        curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT, 10L);
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, &UploadWorker::writeResponse);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response);
        curl_easy_setopt(curl, CURLOPT_LOW_SPEED_LIMIT, 1L);
        curl_easy_setopt(curl, CURLOPT_LOW_SPEED_TIME, config_.timeoutSeconds);

        const auto result = curl_easy_perform(curl);
        long httpCode = 0;
        curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &httpCode);

        const bool ok = result == CURLE_OK && httpCode >= 200 && httpCode < 300;
        if (!ok) {
            RCLCPP_ERROR(logger_, "Upload of %s failed: %s (http=%ld, response=%s)",
                         eventName.c_str(),
                         result != CURLE_OK ? curl_easy_strerror(result) : "server rejected",
                         httpCode, response.substr(0, 200).c_str());
        } else {
            RCLCPP_INFO(logger_, "Uploaded %s (%zu files)",
                        eventName.c_str(), files.size());
        }

        curl_slist_free_all(headers);
        curl_mime_free(mime);
        curl_easy_cleanup(curl);
        return ok;
    }

    static std::string escapeUrl(CURL* curl, const std::string& value) {
        char* escaped = curl_easy_escape(curl, value.c_str(), 0);
        if (!escaped) {
            return value;
        }
        std::string result = escaped;
        curl_free(escaped);
        return result;
    }

    void run() {
        RCLCPP_INFO(logger_, "Upload worker started: root=%s url=%s scan=%lldms",
                    recordRoot_.string().c_str(), config_.url.c_str(),
                    static_cast<long long>(config_.scanPeriod.count()));
        while (true) {
            const auto now = std::chrono::steady_clock::now();
            for (const auto& directory : findUploadCandidates()) {
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    if (stopping_) {
                        return;
                    }
                    const auto state = pending_.find(directory.string());
                    if (state != pending_.end() && now < state->second.nextAttempt) {
                        continue;
                    }
                }

                // 上传在锁外执行, stop() 不必等待网络超时
                if (uploadDirectory(directory)) {
                    std::lock_guard<std::mutex> lock(mutex_);
                    pending_.erase(directory.string());
                    continue;
                }

                std::lock_guard<std::mutex> lock(mutex_);
                if (stopping_) {
                    return;
                }
                auto& state = pending_[directory.string()];
                ++state.attempts;
                if (config_.maxRetries > 0 && state.attempts >= config_.maxRetries) {
                    RCLCPP_ERROR(logger_,
                                 "Upload of %s exceeded %d retries; marked failed",
                                 directory.filename().string().c_str(),
                                 config_.maxRetries);
                    std::ofstream failed(directory / ".upload_failed", std::ios::trunc);
                    failed << "attempts=" << state.attempts << "\n";
                    pending_.erase(directory.string());
                    continue;
                }
                // 指数退避: backoff * 2^(attempts-1), 封顶 64 倍
                const auto delay = config_.retryBackoff *
                    (1LL << std::min(state.attempts - 1, 6));
                state.nextAttempt = now + delay;
                RCLCPP_WARN(logger_, "Upload of %s failed (%d/%d); retry in %lld ms",
                            directory.filename().string().c_str(), state.attempts,
                            config_.maxRetries,
                            static_cast<long long>(delay.count()));
            }

            std::unique_lock<std::mutex> lock(mutex_);
            if (stopping_) {
                return;
            }
            condition_.wait_for(lock, config_.scanPeriod);
        }
    }

    std::filesystem::path recordRoot_;
    Config config_;
    rclcpp::Logger logger_;
    std::map<std::string, RetryState> pending_;
    std::thread worker_;
    std::mutex mutex_;
    std::condition_variable condition_;
    bool stopping_{false};
    bool started_{false};
};
