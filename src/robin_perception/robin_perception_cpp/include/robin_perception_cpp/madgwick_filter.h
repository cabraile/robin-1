#ifndef ROBIN_PERCEPTION_CPP_MADGWICK_FILTER
#define ROBIN_PERCEPTION_CPP_MADGWICK_FILTER

#include <robin_core_cpp/core.h>
#include <robin_firmware_cpp/imu_reading.hpp>
#include <robin_perception_cpp/structures.h>

namespace robin_perception
{

class MadgwickFilter
{

  public:
    MadgwickFilter() = delete;

    MadgwickFilter(const float beta = 0.1f, const float sample_frequency_hz = 30.0);

    void update(const robin_firmware::ImuReading& imu);

    FilteredImu getFilteredImu() const;

  private:
    FilteredImu imu_last_;
    float       beta_;
    float       sample_period_sec_;
    bool        initialied_;
};

} // namespace robin_perception

#endif // ROBIN_PERCEPTION_CPP_MADGWICK_FILTER
