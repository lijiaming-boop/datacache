#pragma once

// 回读共享库：解析事件目录的 manifest.csv / pairs.csv，把落盘的
// .bin / .bin.zst 记录解压并反序列化回 ROS2 消息。
// record_reader 工具与单元测试共用这份实现，保证"写得出就读得回"。

#include <algorithm>
#include <cctype>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <openssl/evp.h>
#include <zstd.h>

#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rclcpp/time.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace record_io {

inline constexpr std::size_t kMaxRecordBytes = 1ULL << 30;

inline bool safeRelativePath(const std::filesystem::path& path) {
    if (path.empty() || path.is_absolute() || path.has_root_name()) {
        return false;
    }
    for (const auto& component : path) {
        const auto text = component.string();
        if (text.empty() || text == "." || text == ".." || text.find(':') != std::string::npos ||
            text.find('\\') != std::string::npos || text.back() == '.' || text.back() == ' ') {
            return false;
        }
        auto stem = text.substr(0, text.find('.'));
        std::transform(stem.begin(), stem.end(), stem.begin(), [](unsigned char character) {
            return static_cast<char>(std::toupper(character));
        });
        static const std::unordered_set<std::string> kWindowsDevices = {
            "CON",  "PRN",  "AUX",  "NUL",  "COM1", "COM2", "COM3", "COM4", "COM5", "COM6", "COM7",
            "COM8", "COM9", "LPT1", "LPT2", "LPT3", "LPT4", "LPT5", "LPT6", "LPT7", "LPT8", "LPT9"};
        if (kWindowsDevices.find(stem) != kWindowsDevices.end()) {
            return false;
        }
    }
    return true;
}

struct ManifestEntry {
    std::string sensor; // camera | lidar
    std::int64_t timestampNs{0};
    std::string file;          // 相对事件目录的记录文件名
    std::string encoding;      // zstd | raw
    std::string convertedFile; // 转换副本相对路径, 可为空
};

struct PairEntry {
    std::uint64_t pairId{0};
    std::string status; // matched | camera_only | lidar_only
    std::int64_t cameraTimestampNs{0};
    std::int64_t lidarTimestampNs{0};
    std::int64_t timeDiffNs{0};
    std::string reason;
};

inline std::vector<std::string> splitCsvLine(const std::string& line) {
    std::vector<std::string> fields;
    std::string current;
    for (const char c : line) {
        if (c == ',') {
            fields.push_back(current);
            current.clear();
        } else if (c != '\r') {
            current.push_back(c);
        }
    }
    fields.push_back(current);
    return fields;
}

// 解析整数: 拒绝空串/残留垃圾/溢出, 代替会抛异常的 std::stoll
inline bool parseI64(const std::string& text, std::int64_t& value) {
    try {
        std::size_t consumed = 0;
        value = std::stoll(text, &consumed);
        return consumed == text.size();
    } catch (const std::exception&) {
        return false;
    }
}

inline bool parseU64(const std::string& text, std::uint64_t& value) {
    if (text.empty() || text.front() == '-') {
        return false;
    }
    try {
        std::size_t consumed = 0;
        value = std::stoull(text, &consumed);
        return consumed == text.size();
    } catch (const std::exception&) {
        return false;
    }
}

// 解析 manifest.csv；文件不存在或只有表头时返回空。
// problems 非空时, 崩溃等留下的撕裂/畸形行记入 problems 而不是抛异常。
inline std::vector<ManifestEntry> readManifest(const std::filesystem::path& eventDirectory,
                                               std::vector<std::string>* problems = nullptr) {
    std::vector<ManifestEntry> entries;
    const auto note = [problems](const std::string& message) {
        if (problems != nullptr) {
            problems->push_back(message);
        }
    };
    std::ifstream input(eventDirectory / "manifest.csv");
    if (!input.is_open()) {
        return entries;
    }
    std::string line;
    std::getline(input, line); // 表头
    std::size_t lineNumber = 1;
    while (std::getline(input, line)) {
        ++lineNumber;
        if (line.empty()) {
            continue;
        }
        const auto fields = splitCsvLine(line);
        if (fields.size() < 4) {
            note("manifest.csv line " + std::to_string(lineNumber) +
                 ": expected at least 4 fields, got " + std::to_string(fields.size()));
            continue;
        }
        ManifestEntry entry;
        entry.sensor = fields[0];
        if (!parseI64(fields[1], entry.timestampNs)) {
            note("manifest.csv line " + std::to_string(lineNumber) + ": invalid timestamp '" +
                 fields[1] + "'");
            continue;
        }
        entry.file = fields[2];
        entry.encoding = fields[3];
        if (fields.size() >= 5) {
            entry.convertedFile = fields[4];
        }
        entries.push_back(std::move(entry));
    }
    return entries;
}

inline std::vector<PairEntry> readPairs(const std::filesystem::path& eventDirectory,
                                        std::vector<std::string>* problems = nullptr) {
    std::vector<PairEntry> entries;
    const auto note = [problems](const std::string& message) {
        if (problems != nullptr) {
            problems->push_back(message);
        }
    };
    std::ifstream input(eventDirectory / "pairs.csv");
    if (!input.is_open()) {
        return entries;
    }
    std::string line;
    std::getline(input, line); // 表头
    std::size_t lineNumber = 1;
    while (std::getline(input, line)) {
        ++lineNumber;
        if (line.empty()) {
            continue;
        }
        const auto fields = splitCsvLine(line);
        if (fields.size() < 6) {
            note("pairs.csv line " + std::to_string(lineNumber) + ": expected 6 fields, got " +
                 std::to_string(fields.size()));
            continue;
        }
        PairEntry entry;
        if (!parseU64(fields[0], entry.pairId)) {
            note("pairs.csv line " + std::to_string(lineNumber) + ": invalid pair_id '" +
                 fields[0] + "'");
            continue;
        }
        entry.status = fields[1];
        if ((!fields[2].empty() && !parseI64(fields[2], entry.cameraTimestampNs)) ||
            (!fields[3].empty() && !parseI64(fields[3], entry.lidarTimestampNs)) ||
            !parseI64(fields[4], entry.timeDiffNs)) {
            note("pairs.csv line " + std::to_string(lineNumber) + ": invalid numeric field");
            continue;
        }
        entry.reason = fields[5];
        entries.push_back(std::move(entry));
    }
    return entries;
}

inline bool readFileBytes(const std::filesystem::path& file, std::vector<std::uint8_t>& bytes) {
    std::ifstream input(file, std::ios::binary | std::ios::ate);
    if (!input.is_open()) {
        return false;
    }
    const auto size = input.tellg();
    if (size <= 0) {
        bytes.clear();
        return true;
    }
    if (static_cast<std::uintmax_t>(size) > kMaxRecordBytes) {
        return false;
    }
    bytes.resize(static_cast<std::size_t>(size));
    input.seekg(0);
    input.read(reinterpret_cast<char*>(bytes.data()), static_cast<std::streamsize>(bytes.size()));
    return static_cast<bool>(input);
}

// zstd 帧内容大小已知时一次性解压，未知时流式兜底
inline bool decompressZstd(const std::uint8_t* data, std::size_t size,
                           std::vector<std::uint8_t>& out, std::string& error) {
    const auto contentSize = ZSTD_getFrameContentSize(data, size);
    if (contentSize != ZSTD_CONTENTSIZE_ERROR && contentSize != ZSTD_CONTENTSIZE_UNKNOWN) {
        if (contentSize > kMaxRecordBytes) {
            error = "decompressed record exceeds safety limit";
            return false;
        }
        out.resize(static_cast<std::size_t>(contentSize));
        const auto decoded = ZSTD_decompress(out.data(), out.size(), data, size);
        if (ZSTD_isError(decoded)) {
            error = ZSTD_getErrorName(decoded);
            return false;
        }
        out.resize(decoded);
        return true;
    }

    ZSTD_DStream* stream = ZSTD_createDStream();
    if (stream == nullptr || ZSTD_isError(ZSTD_initDStream(stream))) {
        ZSTD_freeDStream(stream);
        error = "cannot create zstd decompressor";
        return false;
    }
    out.clear();
    std::vector<std::uint8_t> chunk(1U << 16);
    ZSTD_inBuffer in{data, size, 0};
    while (true) {
        ZSTD_outBuffer outBuffer{chunk.data(), chunk.size(), 0};
        const auto result = ZSTD_decompressStream(stream, &outBuffer, &in);
        if (ZSTD_isError(result)) {
            error = ZSTD_getErrorName(result);
            ZSTD_freeDStream(stream);
            return false;
        }
        if (outBuffer.pos > kMaxRecordBytes - out.size()) {
            error = "decompressed record exceeds safety limit";
            ZSTD_freeDStream(stream);
            return false;
        }
        out.insert(out.end(), chunk.begin(),
                   chunk.begin() + static_cast<std::ptrdiff_t>(outBuffer.pos));
        if (result == 0) {
            break; // 帧解压完成
        }
        if (in.pos == in.size && outBuffer.pos == 0) {
            error = "truncated zstd frame"; // 输入耗尽但无进展
            ZSTD_freeDStream(stream);
            return false;
        }
    }
    ZSTD_freeDStream(stream);
    return true;
}

// 读取一条落盘记录: encoding 为 "zstd" 时自动解压
inline bool loadRecordBytes(const std::filesystem::path& directory, const ManifestEntry& entry,
                            std::vector<std::uint8_t>& bytes, std::string& error) {
    std::vector<std::uint8_t> fileBytes;
    if (!safeRelativePath(entry.file)) {
        error = "unsafe record path: " + entry.file;
        return false;
    }
    if (!readFileBytes(directory / entry.file, fileBytes)) {
        error = "cannot read " + entry.file;
        return false;
    }
    if (entry.encoding == "zstd") {
        return decompressZstd(fileBytes.data(), fileBytes.size(), bytes, error);
    }
    bytes = std::move(fileBytes);
    return true;
}

template <typename Message>
bool deserializeMessage(const std::vector<std::uint8_t>& bytes, Message& message,
                        std::string& error) {
    try {
        rclcpp::SerializedMessage serialized(bytes.size());
        auto& raw = serialized.get_rcl_serialized_message();
        std::memcpy(raw.buffer, bytes.data(), bytes.size());
        raw.buffer_length = bytes.size();
        rclcpp::Serialization<Message>().deserialize_message(&serialized, &message);
        return true;
    } catch (const std::exception& exception) {
        error = exception.what();
        return false;
    }
}

// ---- 完整性校验 ----

struct VerifyOptions {
    std::string sensor;   // "" = 相机与雷达都校验
    std::size_t limit{0}; // 0 = 不限制条数
};

struct VerificationReport {
    std::size_t totalEntries{0};
    std::size_t verifiedEntries{0};
    std::size_t failedEntries{0};
    std::vector<std::string> problems;
    std::vector<std::string> warnings;

    bool ok() const { return failedEntries == 0 && totalEntries > 0 && problems.empty(); }
};

inline bool sha256File(const std::filesystem::path& file, std::string& digest) {
    std::ifstream input(file, std::ios::binary);
    EVP_MD_CTX* context = input ? EVP_MD_CTX_new() : nullptr;
    if (context == nullptr || EVP_DigestInit_ex(context, EVP_sha256(), nullptr) != 1) {
        EVP_MD_CTX_free(context);
        return false;
    }
    std::vector<char> buffer(1U << 16);
    while (input) {
        input.read(buffer.data(), static_cast<std::streamsize>(buffer.size()));
        const auto count = input.gcount();
        if (count > 0 &&
            EVP_DigestUpdate(context, buffer.data(), static_cast<std::size_t>(count)) != 1) {
            EVP_MD_CTX_free(context);
            return false;
        }
    }
    unsigned char hash[EVP_MAX_MD_SIZE];
    unsigned int length = 0;
    const bool ok = input.eof() && EVP_DigestFinal_ex(context, hash, &length) == 1;
    EVP_MD_CTX_free(context);
    if (!ok) {
        return false;
    }
    static constexpr char kHex[] = "0123456789abcdef";
    digest.clear();
    for (unsigned int index = 0; index < length; ++index) {
        digest.push_back(kHex[hash[index] >> 4]);
        digest.push_back(kHex[hash[index] & 0x0f]);
    }
    return true;
}

inline bool verifyChecksumManifest(const std::filesystem::path& directory,
                                   std::vector<std::string>& problems) {
    std::ifstream input(directory / "manifest.sha256");
    if (!input) {
        problems.push_back("manifest.sha256 missing");
        return false;
    }
    std::unordered_map<std::string, std::string> expected;
    std::string line;
    std::size_t lineNumber = 0;
    while (std::getline(input, line)) {
        ++lineNumber;
        const auto separator = line.find("  ");
        if (separator != 64 || line.size() <= separator + 2) {
            problems.push_back("manifest.sha256 line " + std::to_string(lineNumber) +
                               ": malformed entry");
            continue;
        }
        const auto digest = line.substr(0, separator);
        const auto relative = line.substr(separator + 2);
        const bool hex = digest.find_first_not_of("0123456789abcdef") == std::string::npos;
        if (!hex || !safeRelativePath(relative) || !expected.emplace(relative, digest).second) {
            problems.push_back("manifest.sha256 line " + std::to_string(lineNumber) +
                               ": invalid path or digest");
        }
    }
    for (const auto& [relative, wanted] : expected) {
        std::string actual;
        if (!sha256File(directory / relative, actual)) {
            problems.push_back("checksum file cannot be read: " + relative);
        } else if (actual != wanted) {
            problems.push_back("sha256 mismatch: " + relative);
        }
    }
    std::unordered_set<std::string> actualFiles;
    std::error_code iterationError;
    for (std::filesystem::recursive_directory_iterator it(directory, iterationError), end;
         !iterationError && it != end; it.increment(iterationError)) {
        std::error_code entryError;
        const auto name = it->path().filename().string();
        const bool symlink = it->is_symlink(entryError);
        if (!entryError && !symlink && it->is_regular_file(entryError) && !entryError &&
            name != "manifest.sha256" && (name.empty() || name.front() != '.') &&
            it->path().extension() != ".tmp") {
            const auto relative = std::filesystem::relative(it->path(), directory, entryError);
            if (entryError || relative.empty()) {
                problems.push_back("cannot resolve checksum file path");
                continue;
            }
            actualFiles.insert(relative.generic_string());
        }
    }
    if (iterationError) {
        problems.push_back("cannot traverse event directory for checksum verification");
    } else if (actualFiles.size() != expected.size()) {
        problems.push_back("checksum manifest file set mismatch");
    } else {
        for (const auto& relative : actualFiles) {
            if (expected.find(relative) == expected.end()) {
                problems.push_back("checksum manifest missing file: " + relative);
            }
        }
    }
    return !expected.empty() && problems.empty();
}

// 逐条校验: 文件存在 -> 可解压 -> 可反序列化 -> 消息字段自洽
inline VerificationReport verifyEventDirectory(const std::filesystem::path& eventDirectory,
                                               const VerifyOptions& options = {}) {
    VerificationReport report;
    verifyChecksumManifest(eventDirectory, report.problems);
    const std::size_t problemsBefore = report.problems.size();
    const auto entries = readManifest(eventDirectory, &report.problems);
    // 撕裂/畸形行本身就是完整性问题: 即使其余记录完好, 校验也不应通过
    report.failedEntries += report.problems.size() - problemsBefore;
    if (entries.empty()) {
        report.problems.push_back("manifest.csv missing or has no entries");
        return report;
    }

    for (const auto& entry : entries) {
        if (!options.sensor.empty() && entry.sensor != options.sensor) {
            continue;
        }
        if (options.limit != 0 && report.totalEntries >= options.limit) {
            break;
        }
        ++report.totalEntries;
        const auto label = entry.sensor + "@" + std::to_string(entry.timestampNs);

        std::vector<std::uint8_t> bytes;
        std::string error;
        if (!loadRecordBytes(eventDirectory, entry, bytes, error)) {
            ++report.failedEntries;
            report.problems.push_back(label + ": " + error);
            continue;
        }
        if (bytes.empty()) {
            ++report.failedEntries;
            report.problems.push_back(label + ": record is empty");
            continue;
        }

        if (entry.sensor == "camera") {
            sensor_msgs::msg::Image image;
            if (!deserializeMessage(bytes, image, error)) {
                ++report.failedEntries;
                report.problems.push_back(label + ": deserialize failed: " + error);
                continue;
            }
            if (image.width == 0 || image.height == 0 || image.step == 0 ||
                image.encoding.empty() ||
                static_cast<std::uint64_t>(image.step) * image.height > image.data.size() ||
                rclcpp::Time(image.header.stamp).nanoseconds() != entry.timestampNs) {
                ++report.failedEntries;
                report.problems.push_back(label + ": inconsistent image fields");
                continue;
            }
        } else if (entry.sensor == "lidar") {
            sensor_msgs::msg::PointCloud2 cloud;
            if (!deserializeMessage(bytes, cloud, error)) {
                ++report.failedEntries;
                report.problems.push_back(label + ": deserialize failed: " + error);
                continue;
            }
            const auto pointCount = static_cast<std::uint64_t>(cloud.width) * cloud.height;
            if (cloud.fields.empty() || cloud.point_step == 0 ||
                pointCount > cloud.data.size() / cloud.point_step ||
                rclcpp::Time(cloud.header.stamp).nanoseconds() != entry.timestampNs) {
                ++report.failedEntries;
                report.problems.push_back(label + ": inconsistent point cloud fields");
                continue;
            }
        } else {
            ++report.failedEntries;
            report.problems.push_back(label + ": unknown sensor '" + entry.sensor + "'");
            continue;
        }

        ++report.verifiedEntries;
        if (!entry.convertedFile.empty() &&
            (!safeRelativePath(entry.convertedFile) ||
             !std::filesystem::exists(eventDirectory / entry.convertedFile))) {
            report.warnings.push_back(label + ": converted copy missing: " + entry.convertedFile);
        }
    }
    return report;
}

} // namespace record_io
