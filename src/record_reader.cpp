// 回读工具: 检查/导出事件目录中落盘的传感器记录。
//
//   record_reader <event_dir>                       列出记录概要
//   record_reader <event_dir> --verify              完整性校验(损坏返回非零)
//   record_reader <event_dir> --export <dir>        导出为 png / pcd
//   record_reader <event_dir> --sensor camera       只处理相机/雷达
//   record_reader <event_dir> --limit 10            只处理前 N 条

#include "record_io.hpp"

#include <algorithm>
#include <cstring>
#include <filesystem>
#include <iostream>
#include <string>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace {

void printUsage() {
    std::cout <<
        "用法: record_reader <event_dir> [选项]\n"
        "  --verify            校验记录完整性(解压+反序列化+字段自洽), 失败退出码 2\n"
        "  --export <dir>      导出为 png(相机) / pcd(雷达)\n"
        "  --sensor <name>     只处理 camera 或 lidar\n"
        "  --limit <n>         只处理前 n 条\n";
}

bool toSizeT(const char* text, std::size_t& value) {
    try {
        value = static_cast<std::size_t>(std::stoll(text));
        return true;
    } catch (const std::exception&) {
        return false;
    }
}

bool exportImage(const sensor_msgs::msg::Image& message,
                 const std::filesystem::path& target) {
    if (message.height == 0 || message.width == 0 || message.step == 0 ||
        message.step * message.height > message.data.size()) {
        return false;
    }
    cv::Mat image;
    const auto& encoding = message.encoding;
    if (encoding == "bgr8") {
        image = cv::Mat(static_cast<int>(message.height), static_cast<int>(message.width),
                        CV_8UC3, const_cast<unsigned char*>(message.data.data()),
                        message.step);
    } else if (encoding == "rgb8") {
        const cv::Mat rgb(static_cast<int>(message.height),
                          static_cast<int>(message.width), CV_8UC3,
                          const_cast<unsigned char*>(message.data.data()), message.step);
        cv::cvtColor(rgb, image, cv::COLOR_RGB2BGR);
    } else if (encoding == "mono8") {
        image = cv::Mat(static_cast<int>(message.height), static_cast<int>(message.width),
                        CV_8UC1, const_cast<unsigned char*>(message.data.data()),
                        message.step);
    } else if (encoding == "bgra8") {
        const cv::Mat bgra(static_cast<int>(message.height),
                           static_cast<int>(message.width), CV_8UC4,
                           const_cast<unsigned char*>(message.data.data()), message.step);
        cv::cvtColor(bgra, image, cv::COLOR_BGRA2BGR);
    } else if (encoding == "rgba8") {
        const cv::Mat rgba(static_cast<int>(message.height),
                           static_cast<int>(message.width), CV_8UC4,
                           const_cast<unsigned char*>(message.data.data()), message.step);
        cv::cvtColor(rgba, image, cv::COLOR_RGBA2BGR);
    } else {
        return false;
    }
    return cv::imwrite(target.string(), image);
}

bool exportPointCloud(const sensor_msgs::msg::PointCloud2& message,
                      const std::filesystem::path& target) {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(message, cloud);
    return pcl::io::savePCDFileBinary(target.string(), cloud) == 0;
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 2) {
        printUsage();
        return 1;
    }

    const std::filesystem::path eventDirectory = argv[1];
    if (!std::filesystem::is_directory(eventDirectory)) {
        std::cerr << "事件目录不存在: " << eventDirectory << "\n";
        return 1;
    }

    record_io::VerifyOptions options;
    std::filesystem::path exportDir;
    bool wantVerify = false;
    for (int i = 2; i < argc; ++i) {
        const std::string argument = argv[i];
        if (argument == "--verify") {
            wantVerify = true;
        } else if (argument == "--export" && i + 1 < argc) {
            exportDir = argv[++i];
        } else if (argument == "--sensor" && i + 1 < argc) {
            options.sensor = argv[++i];
            if (options.sensor != "camera" && options.sensor != "lidar") {
                std::cerr << "--sensor 只支持 camera 或 lidar\n";
                return 1;
            }
        } else if (argument == "--limit" && i + 1 < argc) {
            if (!toSizeT(argv[++i], options.limit)) {
                std::cerr << "--limit 需要一个整数\n";
                return 1;
            }
        } else {
            std::cerr << "未知参数: " << argument << "\n";
            printUsage();
            return 1;
        }
    }

    // 概要: manifest 与 pairs
    const auto entries = record_io::readManifest(eventDirectory);
    const auto pairs = record_io::readPairs(eventDirectory);
    std::cout << "事件目录: " << eventDirectory.string() << "\n";
    std::cout << "manifest 记录数: " << entries.size() << ", pairs 记录数: "
              << pairs.size() << "\n\n";
    std::cout << "sensor    timestamp(ns)       encoding  size(bytes)  converted\n";

    std::size_t shown = 0;
    for (const auto& entry : entries) {
        if (!options.sensor.empty() && entry.sensor != options.sensor) {
            continue;
        }
        if (options.limit != 0 && shown >= options.limit) {
            break;
        }
        ++shown;
        std::error_code error;
        const auto size = std::filesystem::file_size(eventDirectory / entry.file, error);
        std::cout << entry.sensor << (entry.sensor == "camera" ? "    " : "     ")
                  << entry.timestampNs << "  " << entry.encoding << "      "
                  << (error ? 0 : static_cast<unsigned long long>(size)) << "        "
                  << (entry.convertedFile.empty() ? "-" : entry.convertedFile) << "\n";
    }

    if (!pairs.empty()) {
        std::cout << "\npairs 摘要: ";
        const auto matched = std::count_if(pairs.begin(), pairs.end(),
            [](const record_io::PairEntry& pair) { return pair.status == "matched"; });
        std::cout << matched << " matched, " << (pairs.size() - matched)
                  << " single-sided\n";
    }

    int exitCode = 0;

    // 导出
    if (!exportDir.empty()) {
        std::error_code error;
        std::filesystem::create_directories(exportDir, error);
        if (error) {
            std::cerr << "无法创建导出目录: " << error.message() << "\n";
            return 1;
        }
        std::size_t exported = 0;
        for (const auto& entry : record_io::readManifest(eventDirectory)) {
            if (!options.sensor.empty() && entry.sensor != options.sensor) {
                continue;
            }
            if (options.limit != 0 && exported >= options.limit) {
                break;
            }
            std::vector<std::uint8_t> bytes;
            std::string loadError;
            if (!record_io::loadRecordBytes(eventDirectory, entry, bytes, loadError)) {
                std::cerr << "读取失败 " << entry.file << ": " << loadError << "\n";
                exitCode = 2;
                continue;
            }
            const auto base = entry.sensor + "_" + std::to_string(entry.timestampNs);
            bool ok = false;
            if (entry.sensor == "camera") {
                sensor_msgs::msg::Image image;
                if (record_io::deserializeMessage(bytes, image, loadError)) {
                    ok = exportImage(image, exportDir / (base + ".png"));
                }
            } else {
                sensor_msgs::msg::PointCloud2 cloud;
                if (record_io::deserializeMessage(bytes, cloud, loadError)) {
                    ok = exportPointCloud(cloud, exportDir / (base + ".pcd"));
                }
            }
            if (ok) {
                ++exported;
            } else {
                std::cerr << "导出失败: " << base << " (" << loadError << ")\n";
                exitCode = 2;
            }
        }
        std::cout << "\n导出 " << exported << " 条到 " << exportDir.string() << "\n";
    }

    // 校验 (--verify 时对全部记录生效, 忽略 --limit)
    if (wantVerify) {
        record_io::VerifyOptions verifyOptions;
        verifyOptions.sensor = options.sensor;
        const auto report = record_io::verifyEventDirectory(eventDirectory, verifyOptions);
        std::cout << "\n校验结果: " << report.verifiedEntries << "/"
                  << report.totalEntries << " 条通过";
        if (!report.warnings.empty()) {
            std::cout << " (" << report.warnings.size() << " 条警告)";
        }
        std::cout << "\n";
        for (const auto& problem : report.problems) {
            std::cerr << "  [失败] " << problem << "\n";
        }
        for (const auto& warning : report.warnings) {
            std::cout << "  [警告] " << warning << "\n";
        }
        if (!report.ok()) {
            exitCode = 2;
        }
    }

    return exitCode;
}
