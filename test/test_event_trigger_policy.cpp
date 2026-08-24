#include "event_trigger_policy.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <vector>

TEST(EventTriggerPolicyTest, ParsesUniqueValidConfiguredEvents) {
    const auto events = event_trigger_policy::parseEventNames(
        "collision, hard_brake,manual_capture, collision, bad/event, ,camera.error");
    EXPECT_EQ(events, (std::vector<std::string>{"collision", "hard_brake", "manual_capture",
                                                "camera.error"}));
}

TEST(EventTriggerPolicyTest, ParsesKeyBindingsAndIgnoresMalformedEntries) {
    const auto bindings = event_trigger_policy::parseKeyBindings(
        {"c=collision", " b = hard_brake ", "xx=invalid", "m=bad/event", "no_separator"});
    ASSERT_EQ(bindings.size(), 2U);
    EXPECT_EQ(bindings.at('c'), "collision");
    EXPECT_EQ(bindings.at('b'), "hard_brake");
}

TEST(EventTriggerPolicyTest, SuppressesKeyRepeatUntilAQuietPeriodPasses) {
    using namespace std::chrono_literals;
    event_trigger_policy::QuietPeriodDebouncer debouncer(700ms);
    const auto start = event_trigger_policy::QuietPeriodDebouncer::Clock::time_point{};

    EXPECT_TRUE(debouncer.accept('m', start));
    EXPECT_FALSE(debouncer.accept('m', start + 500ms));
    EXPECT_FALSE(debouncer.accept('m', start + 900ms));
    EXPECT_TRUE(debouncer.accept('m', start + 1700ms));
    EXPECT_TRUE(debouncer.accept('c', start + 1701ms));
}
