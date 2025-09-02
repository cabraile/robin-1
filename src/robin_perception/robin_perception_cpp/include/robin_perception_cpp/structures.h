#ifndef ROBIN_PERCEPTION_CPP_STRUCTURES_H
#define ROBIN_PERCEPTION_CPP_STRUCTURES_H

#include <memory>
#include <robin_core_cpp/core.h>
#include <robin_firmware_cpp/imu_reading.hpp>

namespace robin_perception
{

struct FilteredImu
{
    robin_core::Vector3    acceleration;
    robin_core::Quaternion orientation;
    robin_core::Vector3    orientation_rate_rpy; // Roll, Pitch, Yaw rates in rad/s
};

struct PerceptionInput
{
    std::shared_ptr<robin_firmware::ImuReading> imu;
};

struct PerceptionOutput
{
    FilteredImu imu;
};
} // namespace robin_perception

#endif