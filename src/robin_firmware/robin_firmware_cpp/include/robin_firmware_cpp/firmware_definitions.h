#ifndef ROBIN_FIRMWARE_FIRMWARE_DEFINITIONS_H
#define ROBIN_FIRMWARE_FIRMWARE_DEFINITIONS_H

namespace robin_firmware {

constexpr int TARGET_LOOP_FREQUENCY_HZ = 30;
constexpr int TARGET_LOOP_PERIOD_MS = 1000 / TARGET_LOOP_FREQUENCY_HZ;
constexpr int SERIAL_BAUD_RATE = 9600;
constexpr char *SERIAL_PORT =
    "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0\0";
#ifdef HIGH
constexpr int ENABLE_MOTOR_SHIELD_OUT_VALUE = HIGH;
#else
constexpr int ENABLE_MOTOR_SHIELD_OUT_VALUE = 255;
#endif
constexpr int IMU_SERIAL_START_BYTE = 0xAB;
constexpr int IMU_SHIELD_MPU = 0x68;
constexpr int MPU_6050_ACCEL_FACTOR =
    16384; //< Source:
           // https://www.geekmomprojects.com/gyroscopes-and-accelerometers-on-a-chip/
constexpr int MPU_6050_GYRO_FACTOR = 131;

} // namespace robin_firmware

#endif
