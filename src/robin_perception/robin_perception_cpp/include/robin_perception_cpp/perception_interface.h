#ifndef ROBIN_PERCEPTION_CPP_PERCEPTION_INTERFACE_H
#define ROBIN_PERCEPTION_CPP_PERCEPTION_INTERFACE_H

#include <robin_firmware_cpp/imu_reading.hpp>
#include <robin_perception_cpp/image_processor.h>
#include <robin_perception_cpp/imu_filter.h>
#include <robin_perception_cpp/slam_system.h>
#include <robin_perception_cpp/structures.h>

namespace robin_perception
{

class PerceptionInterface
{

  public:
    PerceptionInterface();

    PerceptionOutput processSensorData(const PerceptionInput& sensor_data);

  private:
    ImuFilter      imu_filter_;
    ImageProcessor image_processor_;
    // SlamSystem slam_system_;
};

} // namespace robin_perception
#endif
