#include "upload_worker.hpp"

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>

namespace {

class RclcppEnvironment : public ::testing::Environment {
public:
    void SetUp() override {
        if (!rclcpp::ok()) {
            int argc = 0;
            rclcpp::init(argc, nullptr);
        }
    }

    void TearDown() override {
        if (rclcpp::ok()) {
            rclcpp::shutdown();
        }
    }
};

[[maybe_unused]] const auto* kRclcppEnvironment =
    ::testing::AddGlobalTestEnvironment(new RclcppEnvironment());

std::string uniqueName(const std::string& prefix) {
    static std::atomic<std::uint64_t> counter{0};
    return prefix + "_" + std::to_string(++counter);
}

class RpcServer {
public:
    explicit RpcServer(bool accept)
        : serviceName_("/" + uniqueName("upload_store_test")),
          node_(std::make_shared<rclcpp::Node>(uniqueName("upload_rpc_server"))), accept_(accept) {
        service_ = node_->create_service<UploadWorker::UploadStore>(
            serviceName_, [this](const std::shared_ptr<UploadWorker::UploadRequest> request,
                                 std::shared_ptr<UploadWorker::UploadResponse> response) {
                handle(*request, *response);
            });
        executor_.add_node(node_);
        thread_ = std::thread([this]() { executor_.spin(); });
    }

    ~RpcServer() {
        executor_.cancel();
        if (thread_.joinable()) {
            thread_.join();
        }
        executor_.remove_node(node_);
    }

    RpcServer(const RpcServer&) = delete;
    RpcServer& operator=(const RpcServer&) = delete;

    const std::string& serviceName() const { return serviceName_; }
    rclcpp::Node* node() const { return node_.get(); }
    int requests() const { return requests_.load(); }

    std::map<std::string, std::vector<std::uint8_t>> files() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return files_;
    }

private:
    void handle(const UploadWorker::UploadRequest& request,
                UploadWorker::UploadResponse& response) {
        ++requests_;
        if (!accept_) {
            response.success = false;
            response.message = "rejected for test";
            return;
        }

        std::lock_guard<std::mutex> lock(mutex_);
        if (request.opcode == UploadWorker::UploadRequest::BEGIN) {
            event_ = request.event_name;
            expectedFiles_ = request.file_count;
            expectedBytes_ = request.total_bytes;
            files_.clear();
            response.success = true;
            response.message = "started";
            return;
        }
        if (request.event_name != event_) {
            response.success = false;
            response.message = "event mismatch";
            return;
        }
        if (request.opcode == UploadWorker::UploadRequest::FILE_CHUNK) {
            auto& bytes = files_[request.file_path];
            if (request.offset != bytes.size() ||
                request.offset + request.data.size() > request.total_bytes) {
                response.success = false;
                response.message = "offset or size mismatch";
                return;
            }
            bytes.insert(bytes.end(), request.data.begin(), request.data.end());
            response.success = true;
            response.message = "stored";
            return;
        }
        if (request.opcode == UploadWorker::UploadRequest::END) {
            std::uint64_t received = 0;
            for (const auto& [path, bytes] : files_) {
                (void)path;
                received += bytes.size();
            }
            response.received_bytes = received;
            response.success = files_.size() == expectedFiles_ && received == expectedBytes_;
            response.message = response.success ? "complete" : "count mismatch";
            return;
        }
        response.success = false;
        response.message = "unknown opcode";
    }

    std::string serviceName_;
    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Service<UploadWorker::UploadStore>::SharedPtr service_;
    rclcpp::executors::SingleThreadedExecutor executor_;
    std::thread thread_;
    bool accept_;
    std::atomic<int> requests_{0};
    mutable std::mutex mutex_;
    std::string event_;
    std::uint32_t expectedFiles_{0};
    std::uint64_t expectedBytes_{0};
    std::map<std::string, std::vector<std::uint8_t>> files_;
};

std::filesystem::path makeTempDir(const std::string& tag) {
    const auto directory =
        std::filesystem::temp_directory_path() / uniqueName("datacache_upload_" + tag);
    std::filesystem::create_directories(directory);
    return directory;
}

void writeFile(const std::filesystem::path& path, const std::string& content) {
    std::error_code error;
    std::filesystem::create_directories(path.parent_path(), error);
    std::ofstream output(path, std::ios::binary | std::ios::trunc);
    output << content;
}

UploadWorker::Config localConfig(const std::string& serviceName) {
    UploadWorker::Config config;
    config.serviceName = serviceName;
    config.timeoutSeconds = 2;
    config.maxRetries = 2;
    config.scanPeriod = std::chrono::milliseconds(30);
    config.retryBackoff = std::chrono::milliseconds(50);
    config.chunkBytes = 4;
    return config;
}

bool waitUntil(const std::function<bool()>& predicate,
               std::chrono::milliseconds timeout = std::chrono::seconds(5)) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
        if (predicate()) {
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return predicate();
}

TEST(UploadWorkerTest, FindsOnlyCompleteUnuploadedDirectories) {
    const auto root = makeTempDir("scan");
    const auto ready = root / "collision_1700000000000000000";
    const auto writing = root / "hard_brake_1700000000000000001";
    const auto uploaded = root / "collision_1700000000000000002";
    const auto failed = root / "collision_1700000000000000003";

    for (const auto* directory : {&ready, &writing, &uploaded, &failed}) {
        writeFile(*directory / "manifest.csv", "header\n");
    }
    writeFile(ready / ".complete", "records=1\n");
    writeFile(uploaded / ".complete", "records=1\n");
    writeFile(uploaded / ".uploaded", "service=x\n");
    writeFile(failed / ".complete", "records=1\n");
    writeFile(failed / ".upload_failed", "attempts=2\n");
    writeFile(root / "not_a_directory", "data");

    auto node = std::make_shared<rclcpp::Node>(uniqueName("upload_scan_client"));
    UploadWorker worker(root, localConfig("/unused_upload_service"), rclcpp::get_logger("test"),
                        node.get());
    const auto candidates = worker.findUploadCandidates();
    ASSERT_EQ(candidates.size(), 1U);
    EXPECT_EQ(candidates[0].filename(), ready.filename());

    std::error_code error;
    std::filesystem::remove_all(root, error);
}

TEST(UploadWorkerTest, RpcTransferChunksFilesAndMarksUploaded) {
    RpcServer server(true);
    const auto root = makeTempDir("rpc_ok");
    const auto event = root / "collision_1700000000000000000";
    writeFile(event / ".complete", "records=3\n");
    writeFile(event / "manifest.csv", "manifest-data");
    writeFile(event / "images" / "camera_1.jpg", "jpeg-bytes");
    writeFile(event / "empty.bin", "");

    UploadWorker worker(root, localConfig(server.serviceName()), rclcpp::get_logger("test"),
                        server.node());
    ASSERT_TRUE(waitUntil([&worker]() { return worker.serviceReady(); }));
    ASSERT_TRUE(worker.uploadDirectory(event));

    EXPECT_TRUE(std::filesystem::exists(event / ".uploaded"));
    EXPECT_TRUE(worker.findUploadCandidates().empty());
    const auto files = server.files();
    ASSERT_EQ(files.size(), 3U);
    EXPECT_EQ(std::string(files.at("manifest.csv").begin(), files.at("manifest.csv").end()),
              "manifest-data");
    EXPECT_EQ(
        std::string(files.at("images/camera_1.jpg").begin(), files.at("images/camera_1.jpg").end()),
        "jpeg-bytes");
    EXPECT_TRUE(files.at("empty.bin").empty());
    EXPECT_EQ(files.count(".complete"), 0U);

    std::error_code error;
    std::filesystem::remove_all(root, error);
}

TEST(UploadWorkerTest, RejectedRpcLeavesDirectoryEligibleForRetry) {
    RpcServer server(false);
    const auto root = makeTempDir("rpc_reject");
    const auto event = root / "hard_brake_1700000000000000000";
    writeFile(event / ".complete", "records=1\n");
    writeFile(event / "lidar_1.bin.zst", "compressed-bytes");

    UploadWorker worker(root, localConfig(server.serviceName()), rclcpp::get_logger("test"),
                        server.node());
    ASSERT_TRUE(waitUntil([&worker]() { return worker.serviceReady(); }));
    EXPECT_FALSE(worker.uploadDirectory(event));
    EXPECT_FALSE(std::filesystem::exists(event / ".uploaded"));
    EXPECT_EQ(worker.findUploadCandidates().size(), 1U);

    std::error_code error;
    std::filesystem::remove_all(root, error);
}

TEST(UploadWorkerTest, BackgroundLoopTransfersCompletedDirectory) {
    RpcServer server(true);
    const auto root = makeTempDir("loop");
    const auto event = root / "collision_1700000000000000000";
    writeFile(event / ".complete", "records=1\n");
    writeFile(event / "camera_1.bin.zst", "compressed-bytes");

    UploadWorker worker(root, localConfig(server.serviceName()), rclcpp::get_logger("test"),
                        server.node());
    ASSERT_TRUE(waitUntil([&worker]() { return worker.serviceReady(); }));
    worker.start();
    EXPECT_TRUE(waitUntil([&event]() { return std::filesystem::exists(event / ".uploaded"); }));
    worker.stop();
    EXPECT_GE(server.requests(), 3); // BEGIN, at least one FILE_CHUNK, END

    std::error_code error;
    std::filesystem::remove_all(root, error);
}

TEST(UploadWorkerTest, RetryExhaustionWritesFailedMarker) {
    RpcServer server(false);
    const auto root = makeTempDir("retry");
    const auto event = root / "collision_1700000000000000000";
    writeFile(event / ".complete", "records=1\n");
    writeFile(event / "camera_1.bin.zst", "compressed-bytes");

    auto config = localConfig(server.serviceName());
    config.maxRetries = 3;
    UploadWorker worker(root, config, rclcpp::get_logger("test"), server.node());
    ASSERT_TRUE(waitUntil([&worker]() { return worker.serviceReady(); }));
    worker.start();
    EXPECT_TRUE(
        waitUntil([&event]() { return std::filesystem::exists(event / ".upload_failed"); }));
    worker.stop();

    EXPECT_EQ(server.requests(), 3);
    EXPECT_TRUE(worker.findUploadCandidates().empty());
    std::ifstream marker(event / ".upload_failed");
    const std::string content((std::istreambuf_iterator<char>(marker)),
                              std::istreambuf_iterator<char>());
    EXPECT_NE(content.find("attempts=3"), std::string::npos);

    std::error_code error;
    std::filesystem::remove_all(root, error);
}

TEST(UploadWorkerTest, ExponentialBackoffDelaysRetries) {
    RpcServer server(false);
    const auto root = makeTempDir("backoff");
    const auto event = root / "collision_1700000000000000000";
    writeFile(event / ".complete", "records=1\n");
    writeFile(event / "camera_1.bin.zst", "compressed-bytes");

    auto config = localConfig(server.serviceName());
    config.maxRetries = 3;
    config.retryBackoff = std::chrono::milliseconds(200);
    config.scanPeriod = std::chrono::milliseconds(20);
    UploadWorker worker(root, config, rclcpp::get_logger("test"), server.node());
    ASSERT_TRUE(waitUntil([&worker]() { return worker.serviceReady(); }));

    const auto started = std::chrono::steady_clock::now();
    worker.start();
    EXPECT_TRUE(waitUntil([&event]() { return std::filesystem::exists(event / ".upload_failed"); },
                          std::chrono::seconds(10)));
    worker.stop();

    const auto elapsed = std::chrono::steady_clock::now() - started;
    EXPECT_GE(std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count(), 600);
    EXPECT_EQ(server.requests(), 3);

    std::error_code error;
    std::filesystem::remove_all(root, error);
}

TEST(UploadWorkerTest, FailedMarkerBecomesRetryableAfterRescanPeriod) {
    RpcServer server(false);
    const auto root = makeTempDir("failed_rescan");
    const auto event = root / "collision_1700000000000000000";
    writeFile(event / ".complete", "records=1\n");
    writeFile(event / "camera_1.bin.zst", "compressed-bytes");

    auto config = localConfig(server.serviceName());
    config.maxRetries = 1;
    config.failedRescanPeriod = std::chrono::milliseconds(80);
    UploadWorker worker(root, config, rclcpp::get_logger("test"), server.node());
    ASSERT_TRUE(waitUntil([&worker]() { return worker.serviceReady(); }));
    worker.start();
    ASSERT_TRUE(
        waitUntil([&event]() { return std::filesystem::exists(event / ".upload_failed"); }));
    worker.stop();

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    const auto candidates = worker.findUploadCandidates();
    ASSERT_EQ(candidates.size(), 1U);
    EXPECT_EQ(candidates[0], event);
    EXPECT_FALSE(std::filesystem::exists(event / ".upload_failed"));

    std::error_code error;
    std::filesystem::remove_all(root, error);
}

} // namespace
