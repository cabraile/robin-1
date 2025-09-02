#include <robin_perception_cpp/perception_interface.h>

namespace robin_perception
{

PerceptionInterface::PerceptionInterface() : imu_filter_(), image_processor_(ImageProcessorSettings{}){}; // TODO: fill

PerceptionOutput PerceptionInterface::processSensorData(const PerceptionInput& sensor_data)
{
    const auto filtered_imu  = imu_filter_.filter(*(sensor_data.imu));
    const auto processed_img = image_processor_.process(*(sensor_data.image));
    // slam_system_.processFrame(processed_img);
    // TODO get optimized pose w.r.t. map
    return {};
}

} // namespace robin_perception
