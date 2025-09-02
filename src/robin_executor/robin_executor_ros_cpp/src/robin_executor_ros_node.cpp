#include <robin_firmware_cpp/interface.hpp>
#include <robin_perception_cpp/perception_interface.h>

#include <rclcpp/rclcpp.hpp>

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
        RCLCPP_INFO(this->get_logger(),
                    "Filtered IMU Reading - Accel: [%.2f, %.2f, %.2f] g, Gyro: [%.2f, %.2f, "
                    "%.2f] deg/s, orientation [qw, qx, qy, qz]: [%.2f, %.2f, %.2f, %.2f]",
                    perception_output.imu.acceleration.x(), perception_output.imu.acceleration.y(),
                    perception_output.imu.acceleration.z(), perception_output.imu.orientation_rate_rpy.x(),
                    perception_output.imu.orientation_rate_rpy.y(), perception_output.imu.orientation_rate_rpy.z(),
                    perception_output.imu.orientation.w(), perception_output.imu.orientation.x(),
                    perception_output.imu.orientation.y(), perception_output.imu.orientation.z());
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
