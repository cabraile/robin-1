#ifndef ROBIN_CORE_VEHICLE_SETINGS_H
#define ROBIN_CORE_VEHICLE_SETINGS_H
#include <cmath>
namespace robin_core
{

constexpr double DEFAULT_ACTUATOR_MAX_VELOCITY_RPM = 70.0;
constexpr double DEFAULT_WHEEL_DIAMETER_MM         = 35.0;
constexpr double DEFAULT_WHEEL_WIDTH_MM            = 13.4;
constexpr double DEFAULT_WHEELS_DISTANCE_CM        = 34.0;
constexpr double DEFAULT_WHEELBASE_CM              = 19.0;

struct VehicleSettings
{
    double active_wheels_distance{DEFAULT_WHEELS_DISTANCE_CM / 100.0}; // Distance between the wheels that are
                                                                       // connected to the motors in meters.
    double wheel_radius{DEFAULT_WHEEL_DIAMETER_MM / (2.0 * 1000.0)};   // Wheel radius in meters
    double wheel_width{DEFAULT_WHEEL_WIDTH_MM / 1000.0};               // The wheel width in meters
    double wheelbase_distance{DEFAULT_WHEELBASE_CM /
                              100.0}; // Distance between the axle and the caster wheel in meters.
    double max_actuator_velocity{DEFAULT_ACTUATOR_MAX_VELOCITY_RPM * (2 * M_PI) /
                                 60.0}; // Actuator max velocity in rad/s
};
} // namespace robin_core

#endif // ROBIN_CORE_VEHICLE_SETINGS_H