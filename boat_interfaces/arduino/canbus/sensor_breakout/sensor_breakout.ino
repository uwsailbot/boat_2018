#include <Wire.h>
#include "mcp2515_iso_tp.h"

#define MCP2515_CS (4)

// I2C addresses
#define ADDR_LSM9DS1_MAG 0x1E
#define ADDR_LSM9DS1_ACCELGYRO 0x6B

MCP2515 mcp2515(MCP2515_CS);
Mcp2515IsoTP can_wrapper(&mcp2515);

iso_tp::Message ane_msg(4, 0b0110000101, 0b0110000100);
iso_tp::Message gps_msg(26, 0b0110000001, 0b0110000000);
iso_tp::Message imu_msg(40, 0b0110000011, 0b0110000010);

template <typename T>
struct vec3 {
  T x;
  T y;
  T z;
} __attribute__((packed));

bool readI2CBuffer(uint8_t addr, uint8_t reg, uint8_t len, uint8_t* buffer) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.endTransmission();
  if (Wire.requestFrom(addr, len) != len) {
    return false;
  }

  for (uint8_t i = 0; i < len; i++) {
    buffer[i] = Wire.read();
  }
  return true;
}

void writeI2CValue(uint8_t addr, uint8_t reg, uint8_t val) {
  return writeI2CBuffer(addr, reg, 1, &val);
}

void writeI2CBuffer(uint8_t addr, uint8_t reg, uint8_t len, uint8_t* buffer) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  for (uint8_t i = 0; i < len; i++) {
    Wire.write(buffer[i]);
  }
  Wire.endTransmission();
}

void setup() {
  Wire.begin();
  Serial.begin(115200);

  mcp2515.reset();
  mcp2515.setBitrate(CAN_125KBPS);
  mcp2515.setFilterMask(MCP2515::MASK0, false, 0x7FF);
  mcp2515.setFilterMask(MCP2515::MASK1, false, 0x7FF);
  mcp2515.setFilter(MCP2515::RXF0, false, gps_msg.rx_id);
  mcp2515.setFilter(MCP2515::RXF1, false, imu_msg.rx_id);
  mcp2515.setNormalMode();

  // Setup the IMU
  writeI2CValue(ADDR_LSM9DS1_MAG, 0x22, 0x00);        // Enable mag
  writeI2CValue(ADDR_LSM9DS1_ACCELGYRO, 0x10, 0xC0);  // Enable gyro
  writeI2CValue(ADDR_LSM9DS1_ACCELGYRO, 0x20, 0xC0);  // Enable accel

  // TODO: Setup GPS, Ane
}

void sendImu() {
  uint8_t buffer[6];
  vec3<float> data;
  size_t size = sizeof(data);

  // Read the mag. Convert from Gauss to Tesla
  // TODO: Quaternion
  if (!readI2CBuffer(ADDR_LSM9DS1_MAG, 0x28, 6, buffer)) return;
  data.x = (buffer[0] | buffer[1] << 8) * 0.00014 / 10000.0;
  data.y = (buffer[2] | buffer[3] << 8) * 0.00014 / 10000.0;
  data.z = (buffer[4] | buffer[5] << 8) * 0.00014 / 10000.0;
  memcpy(imu_msg.data, &data, size);
  imu_msg.data[size + 1] = 0;
  imu_msg.data[size + 2] = 0;

  // Read the gyro. Convert fom deg/sec to rad/sec
  if (!readI2CBuffer(ADDR_LSM9DS1_ACCELGYRO, 0x18, 6, buffer)) return;
  data.x = (buffer[0] | buffer[1] << 8) * 0.00875 / 180.0 * PI;
  data.y = (buffer[2] | buffer[3] << 8) * 0.00875 / 180.0 * PI;
  data.z = (buffer[4] | buffer[5] << 8) * 0.00875 / 180.0 * PI;
  memcpy(imu_msg.data + size + 2, &data, size);

  // Read the accelerometer. Convert from g to m/s^2
  if (!readI2CBuffer(ADDR_LSM9DS1_ACCELGYRO, 0x28, 6, buffer)) return;
  data.x = (buffer[0] | buffer[1] << 8) * 0.000061 * 9.80665;
  data.y = (buffer[2] | buffer[3] << 8) * 0.000061 * 9.80665;
  data.z = (buffer[4] | buffer[5] << 8) * 0.000061 * 9.80665;
  memcpy(imu_msg.data + 2 * size + 2, &data, size);

  can_wrapper.sendMessage(&imu_msg);
}

void loop() {
  can_wrapper.update();

  static unsigned long last_imu_time = millis();
  if ((millis() - last_imu_time) >= 50) {  // 20hz
    sendImu();
    last_imu_time = millis();
  }

  // TODO: Handle GPS, Ane
}
