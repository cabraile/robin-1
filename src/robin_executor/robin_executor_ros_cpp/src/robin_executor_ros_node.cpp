#include <rclcpp/rclcpp.hpp>
#include <robin_firmware_cpp/interface.hpp>

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
    RobinExecutorRosNode() : Node(ROBIN_ROS2_NODE_NAME.data())
    {
        using std::placeholders::_1;
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
    robin_firmware::Interface    firmware_interface_{};
    rclcpp::TimerBase::SharedPtr spin_timer_{};

    void loop()
    {
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
    }
};

} // namespace robin_ros2
} // namespace robin

int main(int argc, char* argv[])
{
    using robin::robin_ros2::RobinExecutorRosNode;

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobinExecutorRosNode>());
    rclcpp::shutdown();
    return 0;
}
