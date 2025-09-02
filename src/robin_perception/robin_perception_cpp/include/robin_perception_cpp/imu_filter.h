#ifndef ROBIN_PERCEPTION_CPP_IMU_FILTER_H
#define ROBIN_PERCEPTION_CPP_IMU_FILTER_H

#include <deque>
#include <robin_core_cpp/core.h>
#include <robin_firmware_cpp/imu_reading.hpp>
#include <robin_perception_cpp/structures.h>

namespace robin_perception
{

class ImuFilter
{
  public:
    ImuFilter() = default;
    FilteredImu filter(const robin_firmware::ImuReading& imu);
};

} // namespace robin_perception

#endif // ROBIN_PERCEPTION_CPP_IMU_FILTER_H
