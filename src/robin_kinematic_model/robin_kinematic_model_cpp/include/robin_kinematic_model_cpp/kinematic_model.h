#ifndef ROBIN_KINEMATIC_MODEL_KINEMATIC_MODEL_H
#define ROBIN_KINEMATIC_MODEL_KINEMATIC_MODEL_H
#include <robin_core_cpp/control_command.h>
#include <robin_core_cpp/twist.h>
#include <robin_core_cpp/vehicle_settings.h>
#include <robin_core_cpp/vehicle_state.h>

namespace robin_kinematic_model
{

using robin_core::ControlCommand;
using robin_core::Twist2d;
using robin_core::VehicleSettings;
using robin_core::VehicleState2d;

/// @brief Returns the velocities in the ego vehicle frame.
/// @details Model based on this paper:
/// https://msl.cs.uiuc.edu/planning/node659.html
Twist2d velocitiesFromControlCommand(const ControlCommand& command, const VehicleSettings& settings);

/// @brief The inverse kinematics from the velocities.
ControlCommand commandFromVelocities(const Twist2d& velocities, const VehicleSettings& settings);

/// @brief
VehicleState2d integrate(const VehicleState2d& state, const double delta_time, const ControlCommand& command,
                         const VehicleSettings& settings, const double target_increment_time_steps);

} // namespace robin_kinematic_model

#endif // ROBIN_KINEMATIC_MODEL_KINEMATIC_MODEL_H