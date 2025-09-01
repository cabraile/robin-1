#ifndef ROBIN_FIRMWARE_MOTOR_COMMAND_H
#define ROBIN_FIRMWARE_MOTOR_COMMAND_H

namespace robin_firmware
{

constexpr char MOTOR_COMMAND_DIRECTION_MASK = 0b00001000;
constexpr char MOTOR_COMMAND_INTENSITY_MASK = 0b00000111;

struct MotorCommand
{
    bool          rotate_forward;
    unsigned char intensity;
};

/// @brief Decodes a command byte.
/// @details Each byte is divided as `dxxxiiii`, where
///   - `d` corresponds to the direction
///   - `x` represents a reserved bit for future usage
///   - `i` represents the intensity of the rotation
/// @param[in] cmd The received command byte.
/// @returns The decoded command.
void decodeMotorCmdBytes(const char cmd, MotorCommand& motor_left, MotorCommand& motor_right)
{
    const char non_scaled_intensity_right = (MOTOR_COMMAND_INTENSITY_MASK & cmd);
    motor_right.intensity                 = (255 * non_scaled_intensity_right) / MOTOR_COMMAND_INTENSITY_MASK;
    motor_right.rotate_forward            = MOTOR_COMMAND_DIRECTION_MASK & cmd;

    const char cmd_shifted               = cmd >> 4;
    const char non_scaled_intensity_left = (MOTOR_COMMAND_INTENSITY_MASK & cmd_shifted);
    motor_left.intensity                 = (255 * non_scaled_intensity_left) / MOTOR_COMMAND_INTENSITY_MASK;
    motor_left.rotate_forward            = MOTOR_COMMAND_DIRECTION_MASK & cmd_shifted;
}

/// @brief Encode a control command to byte. Note that the command only uses 4
/// bits.
/// @details The byte is formated as `xxxxdiii`, where
///   - `x` represents a reserved bit
///   - `d` corresponds to the direction
///   - `i` represents the intensity of the rotation
/// @param[in] cmd The motor command to be encoded.
/// @returns The encoded command.
char encodeMotorCmdToByte(const MotorCommand& cmd)
{
    const char direction_bit  = (cmd.rotate_forward) ? MOTOR_COMMAND_DIRECTION_MASK : 0b00000000;
    const char intensity_byte = MOTOR_COMMAND_INTENSITY_MASK & ((MOTOR_COMMAND_INTENSITY_MASK * cmd.intensity) / 255);
    return direction_bit | intensity_byte;
}

char encodeMotorCmdToByte(const MotorCommand& cmd_left, const MotorCommand& cmd_right)
{
    const char cmd_byte_left  = (0b00001111 & robin_firmware::encodeMotorCmdToByte(cmd_left)) << 4;
    const char cmd_byte_right = (0b00001111 & robin_firmware::encodeMotorCmdToByte(cmd_right));
    return cmd_byte_left | cmd_byte_right;
}

} // namespace robin_firmware

#endif
