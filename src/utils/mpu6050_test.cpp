#include <Arduino.h>
#include <Wire.h>
#include <MahonyAHRS.h>

#define I2C_SDA   17
#define I2C_SCL   18
#define MPU_ADDR  0x68

// MPU6050 registers
#define REG_SMPLRT_DIV    0x19
#define REG_CONFIG        0x1A
#define REG_GYRO_CONFIG   0x1B
#define REG_ACCEL_CONFIG  0x1C
#define REG_ACCEL_CONFIG2 0x1D
#define REG_PWR_MGMT_1    0x6B
#define REG_ACCEL_XOUT_H  0x3B

// At ±2g and ±250 dps
static constexpr float ACC_LSB_PER_G = 16384.0f;
static constexpr float GYRO_LSB_PER_DPS = 131.0f;
static constexpr float SAMPLE_HZ = 100.0f;

Mahony filter;

// gyro bias in deg/s
float gyroBiasX = 0.0f;
float gyroBiasY = 0.0f;
float gyroBiasZ = 0.0f;

void writeReg(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission(true);
}

void readBytes(uint8_t reg, uint8_t *buf, size_t len) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, (uint8_t)len, (uint8_t)true);

  for (size_t i = 0; i < len && Wire.available(); i++) {
    buf[i] = Wire.read();
  }
}

bool readMpu6050(int16_t &ax, int16_t &ay, int16_t &az,
                 int16_t &gx, int16_t &gy, int16_t &gz) {
  uint8_t data[14];
  readBytes(REG_ACCEL_XOUT_H, data, sizeof(data));

  ax = (int16_t)((data[0] << 8) | data[1]);
  ay = (int16_t)((data[2] << 8) | data[3]);
  az = (int16_t)((data[4] << 8) | data[5]);
  gx = (int16_t)((data[8] << 8) | data[9]);
  gy = (int16_t)((data[10] << 8) | data[11]);
  gz = (int16_t)((data[12] << 8) | data[13]);

  return true;
}

void setupMpu6050() {
  writeReg(REG_PWR_MGMT_1, 0x00);   // wake up
  delay(100);

  writeReg(REG_SMPLRT_DIV, 0x04);   // 1kHz / (1 + 4) = 200 Hz internal sample
  writeReg(REG_CONFIG, 0x03);       // DLPF
  writeReg(REG_GYRO_CONFIG, 0x00);  // ±250 dps
  writeReg(REG_ACCEL_CONFIG, 0x00); // ±2g
  writeReg(REG_ACCEL_CONFIG2, 0x03);
}

void calibrateGyro() {
  const int samples = 500;
  int64_t sumX = 0, sumY = 0, sumZ = 0;

  Serial.println("Keep the board still: calibrating gyro bias...");

  for (int i = 0; i < samples; i++) {
    int16_t ax, ay, az, gx, gy, gz;
    readMpu6050(ax, ay, az, gx, gy, gz);
    sumX += gx;
    sumY += gy;
    sumZ += gz;
    delay(2);
  }

  gyroBiasX = (float)sumX / samples / GYRO_LSB_PER_DPS;
  gyroBiasY = (float)sumY / samples / GYRO_LSB_PER_DPS;
  gyroBiasZ = (float)sumZ / samples / GYRO_LSB_PER_DPS;

  Serial.printf("Gyro bias: X=%.3f  Y=%.3f  Z=%.3f deg/s\n",
                gyroBiasX, gyroBiasY, gyroBiasZ);
}

static inline void remapAxes(float &x, float &y, float &z) {
  // First guess for a board mounted 90° rotated in-plane.
  // If roll/pitch are still swapped, change only this block.
  const float rx = +y;
  const float ry = -x;
  const float rz = +z;
  x = rx;
  y = ry;
  z = rz;
}

void setup() {
  Serial.begin(115200);
  delay(1500);

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);

  setupMpu6050();

  filter.begin(SAMPLE_HZ);

  calibrateGyro();

  Serial.println("Roll  Pitch  YawRate");
}

void loop() {
  int16_t rawAx, rawAy, rawAz, rawGx, rawGy, rawGz;
  readMpu6050(rawAx, rawAy, rawAz, rawGx, rawGy, rawGz);

  float ax = (float)rawAx / ACC_LSB_PER_G;
  float ay = (float)rawAy / ACC_LSB_PER_G;
  float az = (float)rawAz / ACC_LSB_PER_G;

  float gx = (float)rawGx / GYRO_LSB_PER_DPS - gyroBiasX;
  float gy = (float)rawGy / GYRO_LSB_PER_DPS - gyroBiasY;
  float gz = (float)rawGz / GYRO_LSB_PER_DPS - gyroBiasZ;

  remapAxes(ax, ay, az);
  remapAxes(gx, gy, gz);

  // MahonyAHRS expects gyro in deg/s; it converts internally.
  filter.updateIMU(gx, gy, gz, ax, ay, az);

  const float roll  = filter.getRoll();
  const float pitch = filter.getPitch();
  const float yawRate = gz;

  Serial.printf("Roll: %7.2f  Pitch: %7.2f  YawRate: %7.2f deg/s\n",
                roll, pitch, yawRate);

  delay(10);
}