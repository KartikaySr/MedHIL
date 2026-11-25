#include "medhil/sim/world.hpp"
#include <gtest/gtest.h>
#include <sstream>
namespace medhil {
TEST(Integration, RegisteredTargetTracking) {
    EXPECT_TRUE(run(Scenario::Nominal, 42).passed(Scenario::Nominal));
}
TEST(Integration, FaultToBrakeLatency) {
    for (auto s : {Scenario::Occlusion, Scenario::Force, Scenario::Joint, Scenario::Fixture,
                   Scenario::Stale, Scenario::Hardware}) {
        auto r = run(s, 42);
        EXPECT_TRUE(r.passed(s)) << name(r.reason);
        EXPECT_GE(r.brake_us, 0);
        if (s == Scenario::Force || s == Scenario::Occlusion || s == Scenario::Hardware) {
            EXPECT_GE(r.brake_us, 60000);
            EXPECT_LE(r.brake_us, 62000);
        }
    }
}
TEST(Verification, SeededTrialsAndByteReplay) {
    for (unsigned seed = 42; seed < 292; ++seed) {
        std::ostringstream a, b;
        ASSERT_TRUE(run(Scenario::MonteCarlo, seed, &a).passed(Scenario::MonteCarlo)) << seed;
        ASSERT_TRUE(run(Scenario::MonteCarlo, seed, &b).passed(Scenario::MonteCarlo)) << seed;
        ASSERT_EQ(a.str(), b.str()) << seed;
    }
}
} // namespace medhil
