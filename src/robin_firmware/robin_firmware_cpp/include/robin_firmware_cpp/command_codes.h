/// @brief Command byte bits mask
/// xxxxxxMx
/// - x: To be defined
/// - M: Activte motor

#ifndef ROBIN_FIRMWARE_COMMAND_CODES_H
#define ROBIN_FIRMWARE_COMMAND_CODES_H

namespace robin_firmware {
constexpr char CMD_ACTIVATE_MOTOR_MASK = 0x02;
} // namespace robin_firmware

#endif
