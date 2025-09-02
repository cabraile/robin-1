#ifndef ROBIN_CORE_VEHICLE_STATE_H
#define ROBIN_CORE_VEHICLE_STATE_H

#include <robin_core_cpp/frames.h>

namespace robin_core
{
struct VehicleState2d
{
    Frame  frame{};
    double x{};
    double y{};
    double yaw{};
};
} // namespace robin_core

#endif // ROBIN_CORE_VEHICLE_STATE_H