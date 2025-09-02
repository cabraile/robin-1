#include <robin_perception_cpp/imu_filter.h>

#include <gtest/gtest.h>

using namespace robin_perception;

TEST(RobinPerception, ImuFilterTest)
{
    ImuFilter imu_filter;

    robin_firmware::ImuReading imu{};
    imu.accel_X_gs         = 0.0f;
    imu.accel_Y_gs         = 0.0f;
    imu.accel_Z_gs         = 1.0f;
    imu.gyro_X_deg_per_sec = 0.0f;
    imu.gyro_Y_deg_per_sec = 0.0f;
    imu.gyro_Z_deg_per_sec = 0.0f;
    auto output            = imu_filter.filter(imu);

    // Check that the orientation is approximately level
    EXPECT_NEAR(output.acceleration.x(), 0.0f, 0.1f);
    EXPECT_NEAR(output.acceleration.y(), 0.0f, 0.1f);
    EXPECT_NEAR(output.acceleration.z(), 0.0f, 0.1f);

    // Now in a rotation in which G is aligned with the X axis
    imu.accel_X_gs         = 1.0f;
    imu.accel_Y_gs         = 0.0f;
    imu.accel_Z_gs         = 0.0f;
    imu.gyro_X_deg_per_sec = 0.0f;
    imu.gyro_Y_deg_per_sec = 0.0f;
    imu.gyro_Z_deg_per_sec = 0.0f;
    output                 = imu_filter.filter(imu);

    // Check that the orientation is approximately level
    EXPECT_NEAR(output.acceleration.x(), 0.0f, 0.1f);
    EXPECT_NEAR(output.acceleration.y(), 0.0f, 0.1f);
    EXPECT_NEAR(output.acceleration.z(), 0.0f, 0.1f);
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}