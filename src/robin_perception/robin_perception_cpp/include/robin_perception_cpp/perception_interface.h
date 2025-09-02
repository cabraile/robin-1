#ifndef ROBIN_PERCEPTION_CPP_PERCEPTION_INTERFACE_H
#define ROBIN_PERCEPTION_CPP_PERCEPTION_INTERFACE_H

#include <robin_firmware_cpp/imu_reading.hpp>
#include <robin_perception_cpp/madgwick_filter.h>
#include <robin_perception_cpp/structures.h>

namespace robin_perception
{

class PerceptionInterface
{

  public:
    PerceptionInterface();

    PerceptionOutput processSensorData(const PerceptionInput& sensor_data);

  private:
    MadgwickFilter madgwick_filter_;
};

} // namespace robin_perception
#endif
