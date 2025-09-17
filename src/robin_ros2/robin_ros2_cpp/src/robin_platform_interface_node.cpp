#include <robin_firmware_cpp/interface.hpp>
#include <robin_kinematic_model_cpp/kinematic_model.h>

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/twist.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/header.hpp>

#include <atomic>
#include <memory>
#include <signal.h>

#include <string_view>
namespace
{
constexpr int QUEUE_SIZE        = 10;
constexpr int SPIN_FREQUENCY_HZ = 30;

constexpr std::string_view ROBIN_ROS2_NODE_NAME  = "robin_platform_interface_node";
constexpr std::string_view ROBIN_CAMERA_FRAME_ID = "camera";
constexpr std::string_view ROBIN_IMU_FRAME_ID    = "imu";

constexpr std::string_view ROBIN_SENSOR_DATA_IMAGE_OUT_TOPIC = "/robin/sensors/image/undistorted/raw";
constexpr std::string_view ROBIN_SENSOR_DATA_IMU_OUT_TOPIC   = "/robin/sensors/imu/unfiltered";
constexpr std::string_view ROBIN_CMD_TWIST_IN_TOPIC          = "/robin/controls/command_twist";

constexpr std::string_view ROBIN_CAMERA_URL = "http://192.168.15.25:5123/video_feed";

} // namespace

namespace robin_ros2
{

// TODO: to load from config
robin_core::VehicleSettings loadVehicleSettings(rclcpp::Node& node)
{
    robin_core::VehicleSettings settings{};
    return settings;
}

class CameraDriver
{

  public:
    CameraDriver()
    {
        video_capture_ptr_ = std::make_unique<cv::VideoCapture>(ROBIN_CAMERA_URL.data(), cv::CAP_FFMPEG);
        if (!video_capture_ptr_->isOpened())
        {
            throw std::runtime_error("Failed to open camera at URL: " + std::string(ROBIN_CAMERA_URL));
        }
    }

    std::optional<cv::Mat> getFrame()
    {
        cv::Mat frame{};
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

class SensorDataPublisher
{

  public:
    SensorDataPublisher(rclcpp::Node& node)
    {
        imu_pub_ptr_ = node.create_publisher<sensor_msgs::msg::Imu>(ROBIN_SENSOR_DATA_IMU_OUT_TOPIC.data(), QUEUE_SIZE);
        cam_pub_ptr_ = node.create_publisher<sensor_msgs::msg::CompressedImage>(
            ROBIN_SENSOR_DATA_IMAGE_OUT_TOPIC.data(), QUEUE_SIZE);
    }

    void publishCameraImage(const cv::Mat& cam_image, const rclcpp::Time& stamp)
    {
        auto img_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", cam_image).toCompressedImageMsg();
        img_msg->header.frame_id = ROBIN_CAMERA_FRAME_ID.data();
        img_msg->header.stamp    = stamp;
        cam_pub_ptr_->publish(*img_msg);
    }

    void publishImu(const robin_firmware::ImuReading& imu, const rclcpp::Time& stamp)
    {
        sensor_msgs::msg::Imu imu_msg{};
        imu_msg.header.frame_id = ROBIN_IMU_FRAME_ID.data();
        imu_msg.header.stamp    = stamp;

        imu_msg.angular_velocity.x = imu.gyro_X_deg_per_sec;
        imu_msg.angular_velocity.y = imu.gyro_Y_deg_per_sec;
        imu_msg.angular_velocity.z = imu.gyro_Z_deg_per_sec;

        imu_msg.linear_acceleration.x = imu.accel_X_gs;
        imu_msg.linear_acceleration.y = imu.accel_Y_gs;
        imu_msg.linear_acceleration.z = imu.accel_Z_gs;

        imu_pub_ptr_->publish(imu_msg);
    }

  private:
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr             imu_pub_ptr_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr cam_pub_ptr_;
};

class RobinExecutorRosNode : public rclcpp::Node
{

  public:
    RobinExecutorRosNode()
        : Node(ROBIN_ROS2_NODE_NAME.data()), sensor_data_pub_(*this), vehicle_settings_(loadVehicleSettings(*this))
    {

        using std::placeholders::_1;
        twist_cmd_sub_ptr_ = this->create_subscription<geometry_msgs::msg::Twist>(
            ROBIN_CMD_TWIST_IN_TOPIC.data(), QUEUE_SIZE,
            std::bind(&RobinExecutorRosNode::twistCommandCallback, this, _1));
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
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_cmd_sub_ptr_;
    CameraDriver                                               cam_driver_{};
    robin_firmware::Interface                                  firmware_interface_{};
    rclcpp::TimerBase::SharedPtr                               spin_timer_{};
    SensorDataPublisher                                        sensor_data_pub_;
    robin_core::VehicleSettings                                vehicle_settings_;

    inline void loop()
    {
        // const auto imu_reading_opt = firmware_interface_.readImu();
        // const auto imu_time        = this->get_clock()->now();
        // if (imu_reading_opt)
        // {
        //     RCLCPP_INFO(this->get_logger(), "Imu exists");
        //     std::cerr << "Imu exists\n";
        //     sensor_data_pub_.publishImu(*imu_reading_opt, imu_time);
        // }

        const auto img_opt  = cam_driver_.getFrame();
        const auto img_time = this->get_clock()->now();
        if (img_opt)
        {
            RCLCPP_INFO(this->get_logger(), "Cam exists");
            sensor_data_pub_.publishCameraImage(*img_opt, img_time);
        }
        else {
            RCLCPP_WARN(this->get_logger(), "Image capture failed.");
        }
    }

    inline void twistCommandCallback(const geometry_msgs::msg::Twist::SharedPtr twist_cmd_in_ptr)
    {
        robin_core::Twist2d twist{};
        twist.linear.x = twist_cmd_in_ptr->linear.x;
        twist.linear.y = twist_cmd_in_ptr->linear.y; //< SHOULD BE ALWAYS ZERO!
        twist.angular  = twist_cmd_in_ptr->angular.z;
        twist.frame    = robin_core::Frame::VEHICLE_AXLE;

        const auto                   cmd = robin_kinematic_model::commandFromVelocities(twist, vehicle_settings_);
        robin_firmware::MotorCommand cmd_left{};
        cmd_left.rotate_forward = (cmd.wheel_velocity_left > 0);
        cmd_left.intensity      = 255 * (cmd.wheel_velocity_left / vehicle_settings_.max_actuator_velocity);
        robin_firmware::MotorCommand cmd_right{};
        cmd_right.rotate_forward = (cmd.wheel_velocity_right > 0);
        cmd_right.intensity      = 255 * (cmd.wheel_velocity_right / vehicle_settings_.max_actuator_velocity);

        // Command logging
        RCLCPP_INFO(this->get_logger(), "Received twist command: %f, %f, %f", twist_cmd_in_ptr->linear.x,
                    twist_cmd_in_ptr->linear.y, twist_cmd_in_ptr->angular.z);
        RCLCPP_INFO(this->get_logger(), "Generated cmd: Left: %d, Right: %d", cmd_left.intensity, cmd_right.intensity);
        firmware_interface_.sendMotorCommands(cmd_right,
                                              cmd_left); // Inverted on purpose.
        RCLCPP_INFO(this->get_logger(), "Sent command!");
    }
};

} // namespace robin_ros2

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
    using robin_ros2::RobinExecutorRosNode;

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
