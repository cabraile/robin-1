#ifndef ROBIN_FIRMWARE_IMU_READING_HPP
#define ROBIN_FIRMWARE_IMU_READING_HPP

#include <string.h>

namespace robin_firmware
{

constexpr int SIZE_OF_FLOAT                 = 4;
constexpr int NUM_IMU_READING_FIELDS        = 6;
constexpr int IMU_READING_BUFFER_SIZE_BYTES = NUM_IMU_READING_FIELDS * SIZE_OF_FLOAT;

struct ImuReading
{
    float accel_X_gs;
    float accel_Y_gs;
    float accel_Z_gs;
    float gyro_X_deg_per_sec;
    float gyro_Y_deg_per_sec;
    float gyro_Z_deg_per_sec;
};

inline void toByteStream(const ImuReading& rec_imu, char* data_buffer_out)
{
    float fields_array[NUM_IMU_READING_FIELDS] = {rec_imu.accel_X_gs,         rec_imu.accel_Y_gs,
                                                  rec_imu.accel_Z_gs,         rec_imu.gyro_X_deg_per_sec,
                                                  rec_imu.gyro_Y_deg_per_sec, rec_imu.gyro_Z_deg_per_sec};

    for (unsigned char field_idx = 0; field_idx < NUM_IMU_READING_FIELDS; ++field_idx)
    {
        memcpy(data_buffer_out + field_idx * SIZE_OF_FLOAT, &fields_array[field_idx], SIZE_OF_FLOAT);
    }
}

inline ImuReading fromByteStream(const char buffer[IMU_READING_BUFFER_SIZE_BYTES])
{
    ImuReading imu_reading{};
    memcpy(&imu_reading.accel_X_gs, buffer + 0 * SIZE_OF_FLOAT, SIZE_OF_FLOAT);
    memcpy(&imu_reading.accel_Y_gs, buffer + 1 * SIZE_OF_FLOAT, SIZE_OF_FLOAT);
    memcpy(&imu_reading.accel_Z_gs, buffer + 2 * SIZE_OF_FLOAT, SIZE_OF_FLOAT);
    memcpy(&imu_reading.gyro_X_deg_per_sec, buffer + 3 * SIZE_OF_FLOAT, SIZE_OF_FLOAT);
    memcpy(&imu_reading.gyro_Y_deg_per_sec, buffer + 4 * SIZE_OF_FLOAT, SIZE_OF_FLOAT);
    memcpy(&imu_reading.gyro_Z_deg_per_sec, buffer + 5 * SIZE_OF_FLOAT, SIZE_OF_FLOAT);
    return imu_reading;
}

} // namespace robin_firmware

#endif
