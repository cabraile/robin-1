#include <robin_perception_cpp/perception_interface.h>

namespace robin_perception
{

PerceptionInterface::PerceptionInterface() : imu_filter_(){};

PerceptionOutput PerceptionInterface::processSensorData(const PerceptionInput& sensor_data)
{
    const auto filtered_imu = imu_filter_.filter(*sensor_data.imu);
    // TODO: have the image processed as well
    slam_system_.processFrame(*(sensor_data.image));
    // TODO get optimized pose w.r.t. map
    return {};
}

} // namespace robin_perception
