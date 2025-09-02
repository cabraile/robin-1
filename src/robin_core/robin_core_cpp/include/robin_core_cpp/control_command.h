#ifndef ROBIN_CORE_CONTROL_COMMAND_H
#define ROBIN_CORE_CONTROL_COMMAND_H
#include <tuple>
namespace robin_core
{
/// @note The wheel center frame is pointed towards the vehicle front.
/// When the velocities have the same signal, they turn to the same direction so
/// that, when positive, the vehicle moves forward and when negative the vehicle
/// moves backwards.
struct ControlCommand
{
    double wheel_velocity_right{};
    double wheel_velocity_left{};
};

struct TimedControlCommand
{
    double         duration_seconds;
    ControlCommand command;
};

} // namespace robin_core

#endif // ROBIN_CORE_CONTROL_COMMAND_H