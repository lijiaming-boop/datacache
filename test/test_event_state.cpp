#include "event_state.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <filesystem>
#include <string>

namespace {

std::filesystem::path makeRoot(const std::string& tag) {
    const auto root = std::filesystem::temp_directory_path() /
                      ("datacache_event_state_" + tag + "_" +
                       std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    std::filesystem::create_directories(root);
    return root;
}

TEST(EventStateTest, ReconcileTurnsCrashPendingIntoExplicitFailure) {
    const auto root = makeRoot("crash");
    const auto event = root / "collision_0_1700000000000000000";
    ASSERT_TRUE(event_state::begin(event, "collision", 1700000000000000000LL));

    const auto result = event_state::reconcile(root);
    EXPECT_EQ(result.recoveredFailures, 1U);
    EXPECT_TRUE(std::filesystem::exists(event / ".failed"));
    EXPECT_FALSE(std::filesystem::exists(event / ".pending"));

    std::error_code error;
    std::filesystem::remove_all(root, error);
}

TEST(EventStateTest, ReconcileClearsStalePendingBesideCompleteMarker) {
    const auto root = makeRoot("complete");
    const auto event = root / "collision_0_1700000000000000001";
    ASSERT_TRUE(event_state::begin(event, "collision", 1700000000000000001LL));
    ASSERT_TRUE(event_state::complete(event, 2));
    ASSERT_TRUE(event_state::writeAtomically(event / ".pending", "stale\n"));

    const auto result = event_state::reconcile(root);
    EXPECT_EQ(result.clearedPending, 1U);
    EXPECT_TRUE(std::filesystem::exists(event / ".complete"));
    EXPECT_FALSE(std::filesystem::exists(event / ".pending"));

    std::error_code error;
    std::filesystem::remove_all(root, error);
}

} // namespace
