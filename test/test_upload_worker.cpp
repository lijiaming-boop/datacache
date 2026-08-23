#include "upload_worker.hpp"

#include <gtest/gtest.h>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>

namespace {

// 进程内最小 HTTP 服务器: 读完整个请求后返回固定状态码, 记录收到的字节
class MiniServer {
public:
    MiniServer(int status, const std::string& body)
        : status_(status), body_(body) {
        listenFd_ = ::socket(AF_INET, SOCK_STREAM, 0);
        if (listenFd_ < 0) {
            throw std::runtime_error("socket() failed");
        }
        int reuse = 1;
        ::setsockopt(listenFd_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

        sockaddr_in address{};
        address.sin_family = AF_INET;
        address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        address.sin_port = 0;
        if (::bind(listenFd_, reinterpret_cast<sockaddr*>(&address),
                   sizeof(address)) != 0 ||
            ::listen(listenFd_, 4) != 0) {
            throw std::runtime_error("bind/listen failed");
        }

        socklen_t length = sizeof(address);
        if (::getsockname(listenFd_, reinterpret_cast<sockaddr*>(&address),
                          &length) != 0) {
            throw std::runtime_error("getsockname failed");
        }
        port_ = ntohs(address.sin_port);

        thread_ = std::thread([this] { serve(); });
    }

    ~MiniServer() { stop(); }

    int port() const { return port_; }
    const std::string& received() const { return received_; }
    int requests() const { return requests_.load(); }

    void stop() {
        if (stopped_.exchange(true)) {
            return;
        }
        // 关闭监听 socket 解除 accept 阻塞
        ::shutdown(listenFd_, SHUT_RDWR);
        ::close(listenFd_);
        if (thread_.joinable()) {
            thread_.join();
        }
    }

private:
    void serve() {
        while (!stopped_) {
            const int client = ::accept(listenFd_, nullptr, nullptr);
            if (client < 0) {
                break;
            }
            handle(client);
            ::close(client);
        }
    }

    void handle(int client) {
        std::string request;
        char buffer[4096];
        while (request.find("\r\n\r\n") == std::string::npos) {
            const auto bytes = ::recv(client, buffer, sizeof(buffer), 0);
            if (bytes <= 0) {
                return;
            }
            request.append(buffer, static_cast<std::size_t>(bytes));
        }

        // 读完 Content-Length 指定的请求体
        std::size_t contentLength = 0;
        const auto position = request.find("Content-Length:");
        if (position != std::string::npos) {
            contentLength = static_cast<std::size_t>(
                std::stoull(request.substr(position + 15)));
        }
        const auto headerEnd = request.find("\r\n\r\n") + 4;
        while (request.size() - headerEnd < contentLength) {
            const auto bytes = ::recv(client, buffer, sizeof(buffer), 0);
            if (bytes <= 0) {
                return;
            }
            request.append(buffer, static_cast<std::size_t>(bytes));
        }

        {
            std::lock_guard<std::mutex> lock(mutex_);
            received_ = request;
            ++requests_;
        }

        const auto statusText = status_ == 200 ? "OK" : "Internal Server Error";
        std::string response = "HTTP/1.1 " + std::to_string(status_) + " " + statusText +
            "\r\nContent-Type: application/json\r\nContent-Length: " +
            std::to_string(body_.size()) + "\r\nConnection: close\r\n\r\n" + body_;
        std::size_t sent = 0;
        while (sent < response.size()) {
            const auto bytes = ::send(client, response.data() + sent,
                                      response.size() - sent, 0);
            if (bytes <= 0) {
                break;
            }
            sent += static_cast<std::size_t>(bytes);
        }
    }

    int status_;
    std::string body_;
    int listenFd_{-1};
    int port_{0};
    std::thread thread_;
    std::atomic<bool> stopped_{false};
    std::atomic<int> requests_{0};
    std::mutex mutex_;
    std::string received_;
};

std::filesystem::path makeTempDir(const std::string& tag) {
    static std::uint64_t counter = 0;
    const auto dir = std::filesystem::temp_directory_path() /
        ("datacache_upload_" + tag + "_" +
         std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()) +
         "_" + std::to_string(++counter));
    std::filesystem::create_directories(dir);
    return dir;
}

void writeFile(const std::filesystem::path& path, const std::string& content) {
    std::error_code error;
    std::filesystem::create_directories(path.parent_path(), error);
    std::ofstream output(path, std::ios::trunc);
    output << content;
}

UploadWorker::Config localConfig(int port) {
    UploadWorker::Config config;
    config.url = "http://127.0.0.1:" + std::to_string(port) + "/upload";
    config.timeoutSeconds = 5;
    config.maxRetries = 2;
    config.scanPeriod = std::chrono::milliseconds(50);
    config.retryBackoff = std::chrono::milliseconds(50);
    return config;
}

TEST(UploadWorkerTest, FindsOnlyCompleteUnuploadedDirectories) {
    const auto root = makeTempDir("scan");
    const auto ready = root / "collision_1700000000000000000";
    const auto writing = root / "hard_brake_1700000000000000001";
    const auto uploaded = root / "collision_1700000000000000002";
    const auto failed = root / "collision_1700000000000000003";
    const auto plainFile = root / "not_a_directory";

    for (const auto* dir : {&ready, &writing, &uploaded, &failed}) {
        std::filesystem::create_directories(*dir);
        writeFile(*dir / "manifest.csv", "sensor,timestamp,file,encoding,converted\n");
    }
    writeFile(ready / ".complete", "records=1\n");
    writeFile(uploaded / ".complete", "records=1\n");
    writeFile(uploaded / ".uploaded", "url=x\n");
    writeFile(failed / ".complete", "records=1\n");
    writeFile(failed / ".upload_failed", "attempts=2\n");
    writeFile(plainFile, "data");

    UploadWorker worker(root, localConfig(80), rclcpp::get_logger("test"));
    const auto candidates = worker.findUploadCandidates();
    ASSERT_EQ(candidates.size(), 1U);
    EXPECT_EQ(candidates[0].filename(), ready.filename());

    std::error_code error;
    std::filesystem::remove_all(root, error);
}

TEST(UploadWorkerTest, UploadDirectoryPostsAndMarksUploaded) {
    MiniServer server(200, "{\"status\":\"ok\"}");

    const auto root = makeTempDir("post_ok");
    const auto event = root / "collision_1700000000000000000";
    writeFile(event / ".complete", "records=2\n");
    writeFile(event / "manifest.csv", "sensor,timestamp,file,encoding,converted\n");
    writeFile(event / "camera_1.bin.zst", "compressed-bytes");
    writeFile(event / "images" / "camera_1.jpg", "jpeg-bytes");

    UploadWorker worker(root, localConfig(server.port()), rclcpp::get_logger("test"));
    ASSERT_TRUE(worker.uploadDirectory(event));

    // 成功标记已写入, 之后再也不会成为候选
    EXPECT_TRUE(std::filesystem::exists(event / ".uploaded"));
    EXPECT_TRUE(worker.findUploadCandidates().empty());

    // 服务端收到了完整请求: 事件名出现在 query, 文件名与内容出现在 multipart 体
    const auto& received = server.received();
    EXPECT_NE(received.find("POST /upload?event=collision_1700000000000000000"),
              std::string::npos);
    EXPECT_NE(received.find("manifest.csv"), std::string::npos);
    EXPECT_NE(received.find("images/camera_1.jpg"), std::string::npos);
    EXPECT_NE(received.find("compressed-bytes"), std::string::npos);
    EXPECT_NE(received.find("jpeg-bytes"), std::string::npos);
    // 标记文件本身不应被上传
    EXPECT_EQ(received.find(".complete"), std::string::npos);

    std::error_code error;
    std::filesystem::remove_all(root, error);
}

TEST(UploadWorkerTest, UploadFailureLeavesNoSuccessMarker) {
    MiniServer server(500, "{\"status\":\"error\"}");

    const auto root = makeTempDir("post_fail");
    const auto event = root / "hard_brake_1700000000000000000";
    writeFile(event / ".complete", "records=1\n");
    writeFile(event / "lidar_1.bin.zst", "compressed-bytes");

    UploadWorker worker(root, localConfig(server.port()), rclcpp::get_logger("test"));
    EXPECT_FALSE(worker.uploadDirectory(event));
    EXPECT_FALSE(std::filesystem::exists(event / ".uploaded"));
    // 未标记成功 → 仍是回传候选(等待重试)
    EXPECT_EQ(worker.findUploadCandidates().size(), 1U);

    std::error_code error;
    std::filesystem::remove_all(root, error);
}

TEST(UploadWorkerTest, BackgroundLoopUploadsThenIdles) {
    MiniServer server(200, "{\"status\":\"ok\"}");

    const auto root = makeTempDir("loop");
    const auto event = root / "collision_1700000000000000000";
    writeFile(event / ".complete", "records=1\n");
    writeFile(event / "camera_1.bin.zst", "compressed-bytes");

    UploadWorker worker(root, localConfig(server.port()), rclcpp::get_logger("test"));
    worker.start();

    // 后台线程应在几个扫描周期内完成上传并写标记
    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (!std::filesystem::exists(event / ".uploaded")) {
        ASSERT_LT(std::chrono::steady_clock::now(), deadline)
            << "background upload did not finish in time";
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    EXPECT_GE(server.requests(), 1);
    worker.stop();

    std::error_code error;
    std::filesystem::remove_all(root, error);
}

}  // namespace
