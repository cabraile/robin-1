#include <robin_perception_cpp/madgwick_filter.h>

#include <cmath>

namespace
{
using robin_core::Quaternion;
using robin_core::Vector3;
using robin_firmware::ImuReading;

constexpr double DEG_TO_RAD                 = M_PI / 180.0;
constexpr double CLOSE_TO_ZERO_TOL          = 1e-6;
constexpr double G_ACCEL_CONST_M_PER_SEC_SQ = 9.80665;
} // namespace

namespace robin_perception
{

MadgwickFilter::MadgwickFilter(const float beta, const float sample_frequency_hz)
    : beta_(beta), sample_period_sec_(1. / sample_frequency_hz), initialied_(false)
{
}

void MadgwickFilter::update(const ImuReading& imu)
{
    const Vector3 accel{imu.accel_X_gs, imu.accel_Y_gs, imu.accel_Z_gs};
    const double  accel_norm = accel.norm();
    // Convert gyroscope degrees/sec to radians/sec
    const Vector3 g(imu.gyro_X_deg_per_sec * DEG_TO_RAD, imu.gyro_Y_deg_per_sec * DEG_TO_RAD,
                    imu.gyro_Z_deg_per_sec * DEG_TO_RAD);

    Quaternion filtered_orientation{};
    if (!initialied_ && accel_norm > CLOSE_TO_ZERO_TOL)
    {
        const auto   a     = accel / accel_norm;
        const double roll  = std::atan2(a.y(), a.z());
        const double pitch = std::atan2(-a.x(), std::sqrt(a.y() * a.y() + a.z() * a.z()));
        const double yaw   = 0.0; // As there is no magnetometer initialize to zero.

        filtered_orientation =
            Quaternion(Eigen::AngleAxisd(yaw, Vector3::UnitZ()) * Eigen::AngleAxisd(pitch, Vector3::UnitY()) *
                       Eigen::AngleAxisd(roll, Vector3::UnitX()));
    }
    else
    {
        auto q = imu_last_.orientation;

        // Rate of change of quaternion from gyroscope
        float q_dot_w = 0.5f * (-q.x() * g.x() - q.y() * g.y() - q.z() * g.z());
        float q_dot_x = 0.5f * (q.w() * g.x() + q.y() * g.z() - q.z() * g.y());
        float q_dot_y = 0.5f * (q.w() * g.y() - q.x() * g.z() + q.z() * g.x());
        float q_dot_z = 0.5f * (q.w() * g.z() + q.x() * g.y() - q.y() * g.x());

        // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
        if (accel_norm > CLOSE_TO_ZERO_TOL)
        {
            const auto a = accel / accel_norm;

            // Auxiliary variables to avoid repeated arithmetic
            const float _2qw = 2.0f * q.w();
            const float _2qx = 2.0f * q.x();
            const float _2qy = 2.0f * q.y();
            const float _2qz = 2.0f * q.z();
            const float _4qw = 4.0f * q.w();
            const float _4qx = 4.0f * q.x();
            const float _4qy = 4.0f * q.y();
            const float _8qx = 8.0f * q.x();
            const float _8qy = 8.0f * q.y();
            const float qwqw = q.w() * q.w();
            const float qxqx = q.x() * q.x();
            const float qyqy = q.y() * q.y();
            const float qzqz = q.z() * q.z();

            // Gradient decent algorithm corrective step
            float s0 = _4qw * qyqy + _2qy * a.x() + _4qw * qxqx - _2qx * a.y();
            float s1 = _4qx * qzqz - _2qz * a.x() + 4.0f * qwqw * q.x() - _2qw * a.y() - _4qx + _8qx * qxqx +
                       _8qx * qyqy + _4qx * a.z();
            float s2 = 4.0f * qwqw * q.y() + _2qw * a.x() + _4qy * qzqz - _2qz * a.y() - _4qy + _8qy * qxqx +
                       _8qy * qyqy + _4qy * a.z();
            float       s3   = 4.0f * qxqx * q.z() - _2qx * a.x() + 4.0f * qyqy * q.z() - _2qy * a.y();
            const float norm = 1. / std::sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
            s0 *= norm;
            s1 *= norm;
            s2 *= norm;
            s3 *= norm;

            // Apply feedback step
            q_dot_w -= beta_ * s0;
            q_dot_x -= beta_ * s1;
            q_dot_y -= beta_ * s2;
            q_dot_z -= beta_ * s3;
        }

        // Integrate rate of change of quaternion to yield quaternion
        q.w() += q_dot_w * sample_period_sec_;
        q.x() += q_dot_x * sample_period_sec_;
        q.y() += q_dot_y * sample_period_sec_;
        q.z() += q_dot_z * sample_period_sec_;
        q.normalize();
        filtered_orientation = q;
    }
    // Filter acceleration
    const auto g_vector_imu = filtered_orientation.toRotationMatrix().transpose() * Vector3{0.0, 0.0, 1.0};
    const auto linear_accel = (accel - g_vector_imu) * G_ACCEL_CONST_M_PER_SEC_SQ;

    // Update last IMU message
    imu_last_ = FilteredImu{linear_accel, filtered_orientation, g};
}

FilteredImu MadgwickFilter::getFilteredImu() const
{
    return imu_last_;
}

} // namespace robin_perception
