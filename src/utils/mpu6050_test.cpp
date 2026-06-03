#include "Arduino.h"
#include <Wire.h>
#include "apps_2/IMUSensor.h"

#define I2C_SDA  17
#define I2C_SCL  18
#define MPU_ADDR 0x68

// Calibration offsets — update from mpu6050_calibration_test output
static constexpr int16_t CAL_XA = -3181;
static constexpr int16_t CAL_YA =  -761;
static constexpr int16_t CAL_ZA =   455;
static constexpr int16_t CAL_XG =    54;
static constexpr int16_t CAL_YG =    -7;
static constexpr int16_t CAL_ZG =     4;

static constexpr float    SAMPLE_HZ   = 200.0f;
static constexpr uint32_t INTERVAL_US = static_cast<uint32_t>(1000000.0f / SAMPLE_HZ);

IMUSensor imu;

void setup()
{
    Serial.begin(115200);
    delay(1500);

    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(400000);

    if (!imu.begin(Wire, MPU_ADDR,
                   CAL_XA, CAL_YA, CAL_ZA,
                   CAL_XG, CAL_YG, CAL_ZG,
                   SAMPLE_HZ,
                   0.05f))
    {
        Serial.println("IMU not found — check wiring");
        while (true) { delay(10); }
    }

    Serial.println("roll\tpitch\tyaw\tax\tay\taz\tgx\tgy\tgz");
}

void loop()
{
    static uint32_t lastUpdate = 0;
    const  uint32_t now        = micros();

    if (now - lastUpdate < INTERVAL_US) return;
    lastUpdate += INTERVAL_US;

    if (!imu.update()) return;

    const IMUSensor::State &s = imu.state();

    Serial.print(s.roll,  2);   Serial.print('\t');
    Serial.print(s.pitch, 2);   Serial.print('\t');
    Serial.print(s.yaw,   2);   Serial.print('\t');
    Serial.print(s.ax,    4);   Serial.print('\t');
    Serial.print(s.ay,    4);   Serial.print('\t');
    Serial.print(s.az,    4);   Serial.print('\t');
    Serial.print(s.gx,    2);   Serial.print('\t');
    Serial.print(s.gy,    2);   Serial.print('\t');
    Serial.println(s.gz,  2);
}
