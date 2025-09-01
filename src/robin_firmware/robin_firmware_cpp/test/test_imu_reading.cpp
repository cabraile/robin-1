#include <gtest/gtest.h>
#include <robin_firmware_cpp/imu_reading.hpp>

TEST(ImuReading, ToFromByteStream)
{
    robin_firmware::ImuReading imu_reading_in{1.0, 2.0, 3.0, 4.0, 5.0, 6.0};
    char                       buffer[robin_firmware::IMU_READING_BUFFER_SIZE_BYTES];
    robin_firmware::toByteStream(imu_reading_in, buffer);
    const robin_firmware::ImuReading imu_reading_out = robin_firmware::fromByteStream(buffer);
    EXPECT_FLOAT_EQ(imu_reading_in.accel_X_gs, imu_reading_out.accel_X_gs);
}