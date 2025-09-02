#include <robin_perception_cpp/imu_filter.h>

#include <cmath>

namespace
{
using robin_core::Quaternion;
using robin_core::Vector3;
using robin_firmware::ImuReading;

constexpr double DEG_TO_RAD                 = M_PI / 180.0;
constexpr double G_ACCEL_CONST_M_PER_SEC_SQ = 9.80665;
} // namespace

namespace robin_perception
{

FilteredImu ImuFilter::filter(const ImuReading& imu)
{
    const Vector3 accel{imu.accel_X_gs, imu.accel_Y_gs, imu.accel_Z_gs};
    const double  accel_norm = accel.norm();
    // Convert gyroscope degrees/sec to radians/sec
    const Vector3 g(imu.gyro_X_deg_per_sec * DEG_TO_RAD, imu.gyro_Y_deg_per_sec * DEG_TO_RAD,
                    imu.gyro_Z_deg_per_sec * DEG_TO_RAD);

    Quaternion   filtered_orientation{};
    const auto   a     = accel / accel_norm;
    const double roll  = std::atan2(a.y(), a.z());
    const double pitch = std::atan2(-a.x(), std::sqrt(a.y() * a.y() + a.z() * a.z()));
    const double yaw   = 0.0; // As there is no magnetometer initialize to zero.

    filtered_orientation =
        Quaternion(Eigen::AngleAxisd(yaw, Vector3::UnitZ()) * Eigen::AngleAxisd(pitch, Vector3::UnitY()) *
                   Eigen::AngleAxisd(roll, Vector3::UnitX()));
    // Filter acceleration
    filtered_orientation.normalize();
    const auto g_vector_imu = filtered_orientation.toRotationMatrix().transpose() * Vector3{0.0, 0.0, 1.0};
    const auto linear_accel = (accel - g_vector_imu) * G_ACCEL_CONST_M_PER_SEC_SQ;

    // Update last IMU message
    return FilteredImu{linear_accel, filtered_orientation, g};
}

} // namespace robin_perception
