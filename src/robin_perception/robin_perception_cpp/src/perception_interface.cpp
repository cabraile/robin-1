#include <robin_perception_cpp/perception_interface.h>

namespace robin_perception
{

PerceptionInterface::PerceptionInterface() : madgwick_filter_(0.1f, 30.0f){};

PerceptionOutput PerceptionInterface::processSensorData(const PerceptionInput& sensor_data)
{
    madgwick_filter_.update(*sensor_data.imu);
    return {madgwick_filter_.getFilteredImu()};
    return {};
}

} // namespace robin_perception