#include <gtest/gtest.h>
#include <robin_core_cpp/core.h>

TEST(Core, CoreSmokeTest)
{
    robin_core::Vector3 v(0.0, 0.0, 1.0);
    EXPECT_DOUBLE_EQ(v.z(), 1.0);
}