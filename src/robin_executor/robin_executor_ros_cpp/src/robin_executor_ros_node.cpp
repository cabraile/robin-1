#include <robin_firmware_cpp/interface.hpp>
#include <robin_perception_cpp/perception_interface.h>
#include <robin_perception_cpp/structures.h>

#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>

#include <atomic>
#include <memory>
#include <signal.h>

#include <string_view>
namespace
{
constexpr int              QUEUE_SIZE           = 10;
constexpr int              SPIN_FREQUENCY_HZ    = 30;
constexpr std::string_view ROBIN_ROS2_NODE_NAME = "robin_executor_ros_node";
} // namespace

namespace robin_executor_ros
{

class CameraDriver
{

  public:
    CameraDriver()
    {
        video_capture_ptr_ = std::make_unique<cv::VideoCapture>(0);
    }

    std::optional<robin_perception::Image> getFrame()
    {
        robin_perception::Image frame{};
        *video_capture_ptr_ >> frame;
        if (frame.empty())
        {
            return std::nullopt;
        }
        return frame;
    }

    ~CameraDriver()
    {
        video_capture_ptr_->release();
    }

  private:
    std::unique_ptr<cv::VideoCapture> video_capture_ptr_;
};

class RobinExecutorRosNode : public rclcpp::Node
{

  public:
    RobinExecutorRosNode() : Node(ROBIN_ROS2_NODE_NAME.data())
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
    CameraDriver                          cam_driver_{};
    robin_firmware::Interface             firmware_interface_{};
    robin_perception::PerceptionInterface perception_interface_{};
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
        const auto img_opt = cam_driver_.getFrame();
        if (!img_opt)
        {
            RCLCPP_DEBUG(this->get_logger(), "No camera frame received.");
        }

        // Perception stack
        const robin_perception::PerceptionInput sensor_data{
            std::make_shared<robin_firmware::ImuReading>(*imu_reading_opt),
            std::make_shared<robin_perception::Image>(*img_opt)};
        const auto perception_output = perception_interface_.processSensorData(sensor_data);
    }
};

} // namespace robin_executor_ros

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
    using robin_executor_ros::RobinExecutorRosNode;

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
