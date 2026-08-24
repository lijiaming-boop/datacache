#include "event_router_policy.hpp"

#include <chrono>

#include <gtest/gtest.h>

using namespace std::chrono_literals;

TEST(EventRouterPolicyTest, RejectsDuplicateTriggerWithinTtl) {
    event_router_policy::EventRouterPolicy policy(10s, 0ms);
    const auto now = event_router_policy::EventRouterPolicy::Clock::now();

    EXPECT_EQ(policy.evaluate("collision", "trigger-1", now),
              event_router_policy::Decision::Accept);
    EXPECT_EQ(policy.evaluate("collision", "trigger-1", now + 1s),
              event_router_policy::Decision::Duplicate);
    EXPECT_EQ(policy.evaluate("collision", "trigger-1", now + 11s),
              event_router_policy::Decision::Accept);
}

TEST(EventRouterPolicyTest, AppliesCooldownPerEvent) {
    event_router_policy::EventRouterPolicy policy(1min, 2s);
    const auto now = event_router_policy::EventRouterPolicy::Clock::now();

    EXPECT_EQ(policy.evaluate("collision", "trigger-1", now),
              event_router_policy::Decision::Accept);
    EXPECT_EQ(policy.evaluate("collision", "trigger-2", now + 1s),
              event_router_policy::Decision::CoolingDown);
    EXPECT_EQ(policy.evaluate("hard_brake", "trigger-3", now + 1s),
              event_router_policy::Decision::Accept);
    EXPECT_EQ(policy.evaluate("collision", "trigger-4", now + 3s),
              event_router_policy::Decision::Accept);
}

TEST(EventRouterPolicyTest, FailedDownstreamAttemptCanBeRetriedWithSameId) {
    event_router_policy::EventRouterPolicy policy(std::chrono::seconds(60),
                                                  std::chrono::seconds(10));
    const auto now = event_router_policy::EventRouterPolicy::Clock::now();
    EXPECT_EQ(policy.evaluate("collision", "retryable", now),
              event_router_policy::Decision::Accept);
    policy.release("retryable");
    EXPECT_EQ(policy.evaluate("collision", "retryable", now + std::chrono::milliseconds(1)),
              event_router_policy::Decision::Accept);
}
