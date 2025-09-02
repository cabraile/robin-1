#include <gtest/gtest.h>

#include <robin_kinematic_model_cpp/kinematic_model.h>

#include <cmath>

constexpr double POSITION_ERROR_TOLERANCE = 1e-5;
namespace robin_kinematic_model
{
TEST(TestVelocities, MovingForward)
{
    VehicleSettings settings{};
    settings.active_wheels_distance = 0.4;
    settings.wheel_radius           = 0.5;
    settings.wheelbase_distance     = 0.1;

    ControlCommand command{};
    command.wheel_velocity_left  = 1.0;
    command.wheel_velocity_right = 1.0;

    const auto velocities = velocitiesFromControlCommand(command, settings);
    EXPECT_GT(velocities.linear.x, 0);
    EXPECT_NEAR(velocities.linear.y, 0, POSITION_ERROR_TOLERANCE);
    EXPECT_NEAR(velocities.angular, 0, POSITION_ERROR_TOLERANCE);
}

TEST(TestVelocities, MovingBackwards)
{
    VehicleSettings settings{};
    settings.active_wheels_distance = 0.4;
    settings.wheel_radius           = 0.5;
    settings.wheelbase_distance     = 0.1;

    ControlCommand command{};
    command.wheel_velocity_left  = -1.0;
    command.wheel_velocity_right = -1.0;

    const auto velocities = velocitiesFromControlCommand(command, settings);
    EXPECT_LT(velocities.linear.x, 0);
    EXPECT_NEAR(velocities.linear.y, 0, POSITION_ERROR_TOLERANCE);
    EXPECT_NEAR(velocities.angular, 0, POSITION_ERROR_TOLERANCE);
}

TEST(TestVelocities, RotateAroundClockwise)
{
    VehicleSettings settings{};
    settings.active_wheels_distance = 0.4;
    settings.wheel_radius           = 0.5;
    settings.wheelbase_distance     = 0.1;

    ControlCommand command{};
    command.wheel_velocity_left  = 1.0;
    command.wheel_velocity_right = -1.0;

    const auto velocities = velocitiesFromControlCommand(command, settings);
    EXPECT_NEAR(velocities.linear.x, 0, POSITION_ERROR_TOLERANCE);
    EXPECT_NEAR(velocities.linear.y, 0, POSITION_ERROR_TOLERANCE);
    EXPECT_LT(velocities.angular, 0);
}

TEST(TestVelocities, RotateAroundCounterClockwise)
{
    VehicleSettings settings{};
    settings.active_wheels_distance = 0.4;
    settings.wheel_radius           = 0.5;
    settings.wheelbase_distance     = 0.1;

    ControlCommand command{};
    command.wheel_velocity_left  = -1.0;
    command.wheel_velocity_right = 1.0;

    const auto velocities = velocitiesFromControlCommand(command, settings);
    EXPECT_NEAR(velocities.linear.x, 0, POSITION_ERROR_TOLERANCE);
    EXPECT_NEAR(velocities.linear.y, 0, POSITION_ERROR_TOLERANCE);
    EXPECT_GT(velocities.angular, 0);
}

TEST(TestIntegration, FacingNorthMovingFront)
{
    VehicleSettings settings{};
    settings.active_wheels_distance = 0.4;
    settings.wheel_radius           = 0.5;
    settings.wheelbase_distance     = 0.1;

    ControlCommand command{};
    command.wheel_velocity_left  = 1.0;
    command.wheel_velocity_right = 1.0;

    VehicleState2d state_init{};
    state_init.x   = 0;
    state_init.y   = 0;
    state_init.yaw = M_PI_2;

    const auto state_new = integrate(state_init, 1.0, command, settings, 0.1);
    EXPECT_NEAR(state_new.x, state_init.x, POSITION_ERROR_TOLERANCE);
    EXPECT_GT(state_new.y, state_init.y);
    EXPECT_NEAR(state_new.yaw, state_init.yaw, POSITION_ERROR_TOLERANCE);
}

TEST(TestIntegration, FacingNorthTurningLeft)
{
    VehicleSettings settings{};
    settings.active_wheels_distance = 0.4;
    settings.wheel_radius           = 0.5;
    settings.wheelbase_distance     = 0.1;

    ControlCommand command{};
    command.wheel_velocity_left  = 0.0;
    command.wheel_velocity_right = 1.0;

    VehicleState2d state_init{};
    state_init.x   = 0;
    state_init.y   = 0;
    state_init.yaw = M_PI_2;

    const auto state_new = integrate(state_init, 1.0, command, settings, 0.1);
    EXPECT_LT(state_new.x, state_init.x);
    EXPECT_GT(state_new.y, state_init.y);
    EXPECT_GT(state_new.yaw, state_init.yaw);
}

TEST(TestIntegration, FacingNorthTurningRight)
{
    VehicleSettings settings{};
    settings.active_wheels_distance = 0.4;
    settings.wheel_radius           = 0.5;
    settings.wheelbase_distance     = 0.1;

    ControlCommand command{};
    command.wheel_velocity_left  = 1.0;
    command.wheel_velocity_right = 0.0;

    VehicleState2d state_init{};
    state_init.x   = 0;
    state_init.y   = 0;
    state_init.yaw = M_PI_2;

    const auto state_new = integrate(state_init, 1.0, command, settings, 0.1);
    EXPECT_GT(state_new.x, state_init.x);
    EXPECT_GT(state_new.y, state_init.y);
    EXPECT_LT(state_new.yaw, state_init.yaw);
}

} // namespace robin_kinematic_model

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}