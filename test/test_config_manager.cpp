#include "config_manager.hpp"

#include <chrono>
#include <filesystem>
#include <fstream>
#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>

namespace {

class TempConfigFile {
public:
    explicit TempConfigFile(const std::string& content) {
        path_ = std::filesystem::temp_directory_path() /
            ("datacache_config_test_" +
             std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
        std::ofstream output(path_);
        output << content;
    }

    ~TempConfigFile() {
        std::error_code error;
        std::filesystem::remove(path_, error);
    }

    std::string path() const { return path_.string(); }

private:
    std::filesystem::path path_;
};

TEST(ConfigManagerTest, ParsesKeyValueWithCommentsAndWhitespace) {
    TempConfigFile file(
        "# 注释行\n"
        "buffer_size = 42\n"
        "  sync_enabled=true  \n"
        "record_directory=/tmp/records\n"
        "\n"
        "trailing_comment=7 # 行尾注释\n");
    ConfigManager manager(rclcpp::get_logger("test"));
    ASSERT_TRUE(manager.loadConfig(file.path()));

    EXPECT_EQ(manager.getConfig("buffer_size"), "42");
    EXPECT_EQ(manager.getConfig("sync_enabled"), "true");
    EXPECT_EQ(manager.getConfig("record_directory"), "/tmp/records");
    EXPECT_EQ(manager.getConfig("trailing_comment"), "7");
    EXPECT_EQ(manager.getConfig("missing_key"), "");
}

TEST(ConfigManagerTest, MissingFileFails) {
    ConfigManager manager(rclcpp::get_logger("test"));
    EXPECT_FALSE(manager.loadConfig("/definitely/not/a/real/path.txt"));
}

TEST(ConfigManagerTest, IntAndGetBoolDefaultsAndInvalidValues) {
    TempConfigFile file(
        "number=17\n"
        "truthy=true\n"
        "falsy=0\n"
        "bad_number=abc\n"
        "bad_bool=maybe\n");
    ConfigManager manager(rclcpp::get_logger("test"));
    ASSERT_TRUE(manager.loadConfig(file.path()));

    EXPECT_EQ(manager.getIntConfig("number", 0), 17);
    EXPECT_EQ(manager.getIntConfig("missing", 99), 99);
    EXPECT_EQ(manager.getIntConfig("bad_number", 5), 5);

    EXPECT_TRUE(manager.getBoolConfig("truthy", false));
    EXPECT_FALSE(manager.getBoolConfig("falsy", true));
    EXPECT_TRUE(manager.getBoolConfig("missing", true));
    EXPECT_FALSE(manager.getBoolConfig("missing", false));
    EXPECT_TRUE(manager.getBoolConfig("bad_bool", true));

    // = 号右侧为空的键应得到空字符串
    EXPECT_EQ(manager.getConfig("bad_number"), "abc");
}

TEST(ConfigManagerTest, ReloadReplacesPreviousContent) {
    TempConfigFile file("key=1\n");
    ConfigManager manager(rclcpp::get_logger("test"));
    ASSERT_TRUE(manager.loadConfig(file.path()));
    EXPECT_EQ(manager.getIntConfig("key", 0), 1);

    {
        std::ofstream output(file.path(), std::ios::trunc);
        output << "other=2\n";
    }
    ASSERT_TRUE(manager.loadConfig(file.path()));
    EXPECT_EQ(manager.getConfig("key"), "");
    EXPECT_EQ(manager.getIntConfig("other", 0), 2);
}

}  // namespace
