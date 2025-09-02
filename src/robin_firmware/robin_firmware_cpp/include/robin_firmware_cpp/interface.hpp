#ifndef ROBIN_FIRMWARE_INTERFACE_HPP
#define ROBIN_FIRMWARE_INTERFACE_HPP
#include <boost/asio.hpp>
#include <mutex>
#include <optional>
#include <robin_firmware_cpp/command_codes.h>
#include <robin_firmware_cpp/firmware_definitions.h>
#include <robin_firmware_cpp/imu_reading.hpp>
#include <robin_firmware_cpp/motor_command.hpp>

namespace robin_firmware
{

class Interface
{
  public:
    Interface() : serial_(io_, SERIAL_PORT)
    {
        // Set options
        serial_.set_option(boost::asio::serial_port_base::baud_rate(SERIAL_BAUD_RATE));
        serial_.set_option(boost::asio::serial_port_base::character_size(8));
        serial_.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
        serial_.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
        serial_.set_option(
            boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
    }

    inline bool sendMotorCommands(const robin_firmware::MotorCommand& left_motor_cmd,
                                  const robin_firmware::MotorCommand& right_motor_cmd)
    {
        std::lock_guard<std::mutex> lock(serial_mutex_);
        const char                  cmd_byte[2]{robin_firmware::CMD_ACTIVATE_MOTOR_MASK,
                                                encodeMotorCmdToByte(left_motor_cmd, right_motor_cmd)};
        boost::asio::write(serial_, boost::asio::buffer(cmd_byte, 2));
        return true;
    }

    inline std::optional<ImuReading> readImu(int max_attempts = 100)
    {
        std::lock_guard<std::mutex> lock(serial_mutex_);
        char                        byte;
        int                         attempts = 0;

        while (attempts < max_attempts)
        {
            boost::asio::read(serial_, boost::asio::buffer(&byte, 1));
            if (static_cast<uint8_t>(byte) == IMU_SERIAL_START_BYTE)
            {
                break;
            }
            attempts++;
        }

        if (attempts == max_attempts)
        {
            return std::nullopt;
        }

        char data[robin_firmware::IMU_READING_BUFFER_SIZE_BYTES];
        boost::asio::read(serial_, boost::asio::buffer(data, sizeof(data)));

        char received_checksum;
        boost::asio::read(serial_, boost::asio::buffer(&received_checksum, 1));

        uint8_t checksum = 0;
        for (const char& b : data)
        {
            checksum ^= static_cast<uint8_t>(b);
        }

        if (checksum != static_cast<uint8_t>(received_checksum))
        {
            return std::nullopt;
        }
        return robin_firmware::fromByteStream(data);
    }

  private:
    boost::asio::io_service  io_;
    boost::asio::serial_port serial_;
    std::mutex               serial_mutex_;
};
} // namespace robin_firmware

#endif // ROBIN_FIRMWARE_INTERFACE_HPP
