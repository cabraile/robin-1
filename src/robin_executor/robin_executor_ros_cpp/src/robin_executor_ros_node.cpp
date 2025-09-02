#include <robin_firmware_cpp/interface.hpp>
#include <robin_perception_cpp/perception_interface.h>

#include <rclcpp/rclcpp.hpp>

#include <atomic>
#include <signal.h>

#include <string_view>
namespace
{
constexpr int              QUEUE_SIZE           = 10;
constexpr int              SPIN_FREQUENCY_HZ    = 30;
constexpr std::string_view ROBIN_ROS2_NODE_NAME = "robin_executor_ros_node";
} // namespace

namespace robin
{

namespace robin_ros2
{

class RobinExecutorRosNode : public rclcpp::Node
{

  public:
    RobinExecutorRosNode() : Node(ROBIN_ROS2_NODE_NAME.data()) //, perception_interface_()
    {
        spin_timer_ = this->create_wall_timer(std::chrono::milliseconds(1000 / SPIN_FREQUENCY_HZ),
                                              std::bind(&RobinExecutorRosNode::loop, this));
    }

    ~RobinExecutorRosNode()
    {
        firmware_interface_.sendMotorCommands({}, {});
    }

    RobinExecutorRosNode(const RobinExecutorRosNode&)            = delete;
    RobinExecutorRosNode& operator=(const RobinExecutorRosNode&) = delete;
    RobinExecutorRosNode(RobinExecutorRosNode&&)                 = delete;
    RobinExecutorRosNode& operator=(RobinExecutorRosNode&&)      = delete;

  private:
    robin_firmware::Interface             firmware_interface_{};
    robin_perception::PerceptionInterface perception_interface_;
    rclcpp::TimerBase::SharedPtr          spin_timer_{};

    void loop()
    {
        // Input receiver
        const auto imu_reading_opt = firmware_interface_.readImu();
        if (!imu_reading_opt)
        {
            RCLCPP_WARN(this->get_logger(), "No IMU reading available. Skipping iteration.");
            return;
        }
        const auto imu_reading = *imu_reading_opt;
        RCLCPP_INFO(this->get_logger(),
                    "IMU Reading - Accel: [%.2f, %.2f, %.2f] g, Gyro: [%.2f, %.2f, "
                    "%.2f] deg/s",
                    imu_reading.accel_X_gs, imu_reading.accel_Y_gs, imu_reading.accel_Z_gs,
                    imu_reading.gyro_X_deg_per_sec, imu_reading.gyro_Y_deg_per_sec, imu_reading.gyro_Z_deg_per_sec);

        // Perception stack
        const robin_perception::PerceptionInput sensor_data{std::make_shared<robin_firmware::ImuReading>(imu_reading)};

        const auto perception_output = perception_interface_.processSensorData(sensor_data);

        const auto imu_rpy = perception_output.imu.orientation.toRotationMatrix().eulerAngles(0, 1, 2) * 180.0 / M_PI;

        RCLCPP_INFO(this->get_logger(),
                    "Filtered IMU Reading - Accel: [%.2f, %.2f, %.2f] g, Gyro: [%.2f, %.2f, "
                    "%.2f] deg/s, orientation [r,p,y]: [%.2f, %.2f, %.2f] deg",
                    perception_output.imu.acceleration.x(), perception_output.imu.acceleration.y(),
                    perception_output.imu.acceleration.z(), perception_output.imu.orientation_rate_rpy.x(),
                    perception_output.imu.orientation_rate_rpy.y(), perception_output.imu.orientation_rate_rpy.z(),
                    imu_rpy[0], imu_rpy[1], imu_rpy[2]);
    }
};

} // namespace robin_ros2
} // namespace robin

std::atomic_bool g_shutdown_requested{false};

void signal_handler(int signal)
{
    if (signal == SIGINT)
    {
        g_shutdown_requested = true;
        rclcpp::shutdown();
    }
}

int main(int argc, char* argv[])
{
    using robin::robin_ros2::RobinExecutorRosNode;

    // Register signal handler
    signal(SIGINT, signal_handler);

    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobinExecutorRosNode>();
    while (rclcpp::ok() && !g_shutdown_requested)
    {
        rclcpp::spin_some(node);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    rclcpp::shutdown();
    return 0;
}
