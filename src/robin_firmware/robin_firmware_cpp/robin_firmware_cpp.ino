#include "include/robin_firmware_cpp/command_codes.h"
#include "include/robin_firmware_cpp/firmware_definitions.h"
#include "include/robin_firmware_cpp/imu_reading.hpp"
#include "include/robin_firmware_cpp/motor_command.hpp"

#include <Wire.h>

// MOTOR SHIELD PINOUT
/////////////////////////////////////////
constexpr int MOTOR_LEFT_PIN_1 = 4;
constexpr int MOTOR_LEFT_PIN_2 = 5;
constexpr int MOTOR_RIGHT_PIN_1 = 6;
constexpr int MOTOR_RIGHT_PIN_2 = 7;
constexpr int ENABLE_MOTOR_SHIELD_PIN = 8;
/////////////////////////////////////////

// IMU SHIELD PINOUT
/////////////////////////////////////////
constexpr int IMU_SCL_PIN = A5;
constexpr int IMU_SDA_PIN = A4;
/////////////////////////////////////////

namespace robin_firmware {

void setupMotorShield() {
  pinMode(MOTOR_LEFT_PIN_1, OUTPUT);
  pinMode(MOTOR_LEFT_PIN_2, OUTPUT);
  pinMode(MOTOR_RIGHT_PIN_1, OUTPUT);
  pinMode(MOTOR_RIGHT_PIN_2, OUTPUT);
  pinMode(ENABLE_MOTOR_SHIELD_PIN, OUTPUT);
}

void setupImuShield() {
  pinMode(IMU_SCL_PIN, INPUT);
  pinMode(IMU_SDA_PIN, INPUT);
  Wire.begin();
  Wire.beginTransmission(IMU_SHIELD_MPU);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  Wire.beginTransmission(IMU_SHIELD_MPU);
  Wire.write(0x6B); // Power management register
  Wire.write(0);    // Wake up the MPU-6050
  Wire.endTransmission(true);
}

void sendSensorData(const ImuReading &rec_imu) {
  char data_buffer[IMU_READING_BUFFER_SIZE_BYTES];
  toByteStream(rec_imu, data_buffer);
  Serial.write(IMU_SERIAL_START_BYTE);
  uint8_t checksum = 0;
  for (int i = 0; i < IMU_READING_BUFFER_SIZE_BYTES; i++) {
    Serial.write(data_buffer[i]);
    checksum ^= data_buffer[i];
  }
  Serial.write(checksum);
}

void activateMotor(const MotorCommand &cmd_motor, const int pin_1,
                   const int pin_2) {
  const int target_signal_pin = (cmd_motor.rotate_forward) ? pin_1 : pin_2;
  const int target_low_pin = (cmd_motor.rotate_forward) ? pin_2 : pin_1;
  digitalWrite(target_signal_pin, cmd_motor.intensity);
  digitalWrite(target_low_pin, LOW);
}

void activateMotors(const MotorCommand &cmd_motor_left,
                    const MotorCommand &cmd_motor_right) {
  digitalWrite(ENABLE_MOTOR_SHIELD_PIN, ENABLE_MOTOR_SHIELD_OUT_VALUE);
  activateMotor(cmd_motor_left, MOTOR_LEFT_PIN_1, MOTOR_LEFT_PIN_2);
  activateMotor(cmd_motor_right, MOTOR_RIGHT_PIN_1, MOTOR_RIGHT_PIN_2);
}

ImuReading readImu() {
  Wire.beginTransmission(IMU_SHIELD_MPU);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(IMU_SHIELD_MPU, 14, true); // Read 14 bytes, including the temperature

  ImuReading rec_imu{};
  int16_t ax = (Wire.read() << 8) | Wire.read();
  int16_t ay = (Wire.read() << 8) | Wire.read();
  int16_t az = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read(); // Ignore temperature
  int16_t gx = (Wire.read() << 8) | Wire.read();
  int16_t gy = (Wire.read() << 8) | Wire.read();
  int16_t gz = (Wire.read() << 8) | Wire.read();

  rec_imu.accel_X_gs = ax / (float)MPU_6050_ACCEL_FACTOR;
  rec_imu.accel_Y_gs = ay / (float)MPU_6050_ACCEL_FACTOR;
  rec_imu.accel_Z_gs = az / (float)MPU_6050_ACCEL_FACTOR;
  rec_imu.gyro_X_deg_per_sec = gx / (float)MPU_6050_GYRO_FACTOR;
  rec_imu.gyro_Y_deg_per_sec = gy / (float)MPU_6050_GYRO_FACTOR;
  rec_imu.gyro_Z_deg_per_sec = gz / (float)MPU_6050_GYRO_FACTOR;

  return rec_imu;
}

struct ParsedCommand {
  bool activate_motors{false};
  MotorCommand activate_motor_left_params{0, 0};
  MotorCommand activate_motor_right_params{0, 0};
};

ParsedCommand receiveCommandsFromSerial() {
  if (!Serial.available()) {
    return {};
  }

  const char cmd_type_byte = Serial.read();
  ParsedCommand parsed_command{};
  parsed_command.activate_motors =
      (CMD_ACTIVATE_MOTOR_MASK & cmd_type_byte);

  if (!parsed_command.activate_motors) {
    return parsed_command;
  }

  const char cmd_encoded_motor_byte = Serial.read();
  decodeMotorCmdBytes(cmd_encoded_motor_byte,
                      parsed_command.activate_motor_left_params,
                      parsed_command.activate_motor_right_params);
  return parsed_command;
}

} // namespace robin_firmware

void setup() {
  robin_firmware::setupMotorShield();
  robin_firmware::setupImuShield();
  Serial.begin(robin_firmware::SERIAL_BAUD_RATE);
}

void loop() {
  delay(robin_firmware::TARGET_LOOP_PERIOD_MS);

  // Communication-related
  const robin_firmware::ParsedCommand received_command =
      robin_firmware::receiveCommandsFromSerial();

  // Send IMU data
  const robin_firmware::ImuReading imu_data = robin_firmware::readImu();
  robin_firmware::sendSensorData(imu_data);

  // Actuation
  if (received_command.activate_motors) {
    robin_firmware::activateMotors(
        received_command.activate_motor_left_params,
        received_command.activate_motor_right_params);
  }
}
