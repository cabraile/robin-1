#include <robin_perception_cpp/perception_interface.h>

#include <gtest/gtest.h>

using namespace robin_perception;

TEST(PerceptionInterfaceTest, ProcessSensorData)
{
    PerceptionInterface perception_interface;

    PerceptionInput input{};
    input.imu                     = std::make_shared<robin_firmware::ImuReading>();
    input.imu->accel_X_gs         = 0.0f;
    input.imu->accel_Y_gs         = 0.0f;
    input.imu->accel_Z_gs         = 1.0f;
    input.imu->gyro_X_deg_per_sec = 0.0f;
    input.imu->gyro_Y_deg_per_sec = 0.0f;
    input.imu->gyro_Z_deg_per_sec = 0.0f;
    PerceptionOutput output       = perception_interface.processSensorData(input);

    // Check that the orientation is approximately level
    EXPECT_NEAR(output.imu.acceleration.x(), 0.0f, 0.1f);
    EXPECT_NEAR(output.imu.acceleration.y(), 0.0f, 0.1f);
    EXPECT_NEAR(output.imu.acceleration.z(), 0.0f, 0.1f);

    // Now in a rotation in which G is aligned with the X axis
    input.imu->accel_X_gs         = 1.0f;
    input.imu->accel_Y_gs         = 0.0f;
    input.imu->accel_Z_gs         = 0.0f;
    input.imu->gyro_X_deg_per_sec = 0.0f;
    input.imu->gyro_Y_deg_per_sec = 0.0f;
    input.imu->gyro_Z_deg_per_sec = 0.0f;
    output                        = perception_interface.processSensorData(input);

    // Check that the orientation is approximately level
    EXPECT_NEAR(output.imu.acceleration.x(), 0.0f, 0.1f);
    EXPECT_NEAR(output.imu.acceleration.y(), 0.0f, 0.1f);
    EXPECT_NEAR(output.imu.acceleration.z(), 0.0f, 0.1f);
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}