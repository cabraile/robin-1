#ifndef ROBIN_PERCEPTION_CPP_STRUCTURES_H
#define ROBIN_PERCEPTION_CPP_STRUCTURES_H

#include <memory>
#include <opencv2/opencv.hpp>
#include <robin_core_cpp/core.h>
#include <robin_firmware_cpp/imu_reading.hpp>

namespace robin_perception
{

using Image = cv::Mat;

struct FilteredImu
{
    robin_core::Vector3    acceleration;
    robin_core::Quaternion orientation;
    robin_core::Vector3    orientation_rate_rpy; // Roll, Pitch, Yaw rates in rad/s
};

struct PerceptionInput
{
    std::shared_ptr<robin_firmware::ImuReading> imu;
    std::shared_ptr<Image>                      image;
};

struct PerceptionOutput
{
};

} // namespace robin_perception

#endif