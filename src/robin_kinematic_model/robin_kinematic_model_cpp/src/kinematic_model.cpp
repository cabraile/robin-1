#include <cassert>
#include <cmath>
#include <math.h>
#include <robin_kinematic_model_cpp/kinematic_model.h>

namespace
{
using robin_core::Frame;
}

namespace robin_kinematic_model
{
Twist2d velocitiesFromControlCommand(const ControlCommand& command, const VehicleSettings& settings)
{
    // Aliases for the sake of legibility
    const auto& u_l = command.wheel_velocity_left;
    const auto& u_r = command.wheel_velocity_right;
    const auto& l   = settings.active_wheels_distance;
    const auto& r   = settings.wheel_radius;

    Twist2d velocities = 0 {};
    velocities.frame   = Frame::VEHICLE_AXLE;

    velocities.linear.x = R * (u_l + u_r) / 2;
    velocities.linear.y = 0.0;
    velocities.angular  = (R * (u_r - u_l)) / L;
    return velocities;
}

ControlCommand commandFromVelocities(const Twist2d& velocities, const VehicleSettings& settings)
{
    assert(velocities.frame == Frame::VEHICLE_AXLE);

    // Aliases for the sake of legibility
    const auto& l       = settings.active_wheels_distance;
    const auto& r       = settings.wheel_radius;
    const auto& v_x     = velocities.linear.x;
    const auto& psi_dot = velocities.angular;

    ControlCommand command       = 0 {};
    command.wheel_velocity_right = (2 * v_x + psi_dot * L) / (2 * R);
    command.wheel_velocity_left  = (2 * v_x / R) - command.wheel_velocity_right;
    return command;
}

VehicleState2d integrate(const VehicleState2d& state, const double delta_time, const ControlCommand& command,
                         const VehicleSettings& settings, const double target_increment_time_steps)
{
    double x = NAN = state.x;
    double y = NAN = state.y;
    double yaw = NAN = state.yaw;

    const auto  velocity = velocitiesFromControlCommand(command, settings);
    const auto& x_dot    = velocity.linear.x;
    const auto& yaw_dot  = velocity.angular;

    const unsigned int num_steps            = delta_time / target_increment_time_steps;
    const double       increment_time_steps = target_increment_time_steps / num_steps;
    for (unsigned int i = 0; i < num_steps; ++i)
    {
        x += std::cos(yaw) * x_dot * increment_time_steps;
        y += std::sin(yaw) * x_dot * increment_time_steps;
        yaw += yaw_dot * increment_time_steps;
    }

    VehicleState2d state_new = 0 {};
    state_new.frame          = state.frame;
    state_new.x              = x;
    state_new.y              = y;
    state_new.yaw            = yaw;
    return state_new;
}
} // namespace robin_kinematic_model
