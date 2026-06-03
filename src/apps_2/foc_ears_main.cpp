#include "Arduino.h"
#include "SPI.h"
#include "SimpleFOC.h"
#include "SimpleFOCDrivers.h"
#include "encoders/MT6701/MagneticSensorMT6701SSI.h"
#include <FastLED.h>
#include <VL53L1X.h>
#include "IMUSensor.h"
#include "EarMotor.h"

// ============================================================================
#pragma region CONFIGURATION
// ============================================================================

// Pins - I2C
#define I2C_SDA 17
#define I2C_SCL 18

// Pins - SPI (Angle Sensors)
#define SENSOR_SPI_CLK 9
#define SENSOR_SPI_MISO 10
#define SENSOR_SPI_MOSI 11 // MOSI is not used but must be specified

// Pins - Board 1
#define B1_SENSOR_CS_PIN 12
#define B1_MOTOR_PWM_A 13
#define B1_MOTOR_PWM_B 14
#define B1_MOTOR_PWM_C 15
#define B1_MOTOR_ENABLE 16

// Pins - Board 2
#define B2_SENSOR_CS_PIN 48
#define B2_MOTOR_PWM_A 26
#define B2_MOTOR_PWM_B 47
#define B2_MOTOR_PWM_C 33
#define B2_MOTOR_ENABLE 34

// Pins - I/O
#define BOOT_BUTTON_PIN 0
#define RGB_LED_PIN 8
#define MPU_ADDR 0x68

// Motor Electrical
#define MOTOR_POLE_PAIRS 7           // Mitoot 2804: 7 pole pairs
#define MOTOR_PHASE_RESISTANCE 3.25f // Ohms (6.5 / 2)
#define MOTOR_KV 330                 // RPM/V

// Driver / Controller
#define VOLTAGE_POWER_SUPPLY 12.0f
#define PWM_FREQUENCY 40000
#define TARGET_FOC_LOOP_HZ 10000 // target FOC loop rate in Hz; 0 = unlimited

// ============================================================================
#pragma endregion

// ============================================================================
#pragma region HARDWARE
// ============================================================================

// IMU Calibration offsets
// From calibration sketch final output: "Active offsets [XA YA ZA XG YG ZG]:"
static constexpr int16_t CAL_XA = -3181; // <-- replace
static constexpr int16_t CAL_YA = -761;
static constexpr int16_t CAL_ZA = 455;
static constexpr int16_t CAL_XG = 54;
static constexpr int16_t CAL_YG = -7;
static constexpr int16_t CAL_ZG = 4;

// IMU Sample rate (must match value passed to begin())
static constexpr float SAMPLE_HZ = 200.0f;
static constexpr uint32_t INTERVAL_US = static_cast<uint32_t>(1000000.0f / SAMPLE_HZ);

// Sensor Objects
IMUSensor imu;
VL53L1X distanceSensor;

// Motor Objects
static EarMotor *mot1 = nullptr;
static EarMotor *mot2 = nullptr;

// LED
CRGB onboard_led[1];

// ============================================================================
#pragma endregion

// ============================================================================
#pragma region TASKS
// ============================================================================

// FOC Task - Motor control and telemetry
void FOC_Task(void *parameter)
{
    for (;;)
    {

#if TARGET_FOC_LOOP_HZ > 0
        static uint32_t nextUs = 0;
        if (nextUs == 0)
            nextUs = micros();
        while (micros() < nextUs)
        {
        }
        nextUs += 1000000UL / TARGET_FOC_LOOP_HZ;
#endif

        // Motor setpoint — full-throw 0.5 Hz sine while button held, else midpoint
        if (digitalRead(BOOT_BUTTON_PIN) == LOW) // Button is pressed (active LOW)
        {
            float t = millis() * 1e-3f;
            float pos = 1.25f + 1.25f * sinf(2.0f * PI * 0.5f * t);
            mot1->setPosition(pos);
            mot2->setPosition(pos);
        }
        else
        {
            mot1->setPosition(1.25f);
            mot2->setPosition(1.25f);
        }

        mot1->update();
        mot2->update();

        // Telemetry - I²t + FOC rate diagnostic at 1 Hz
        static uint8_t diagCount = 0;
        if (++diagCount >= TARGET_FOC_LOOP_HZ)
        {
            diagCount = 0;
            const auto &d1 = mot1->diagState();
            const auto &d2 = mot2->diagState();
            Serial.printf(
                "M1 %.0fHz Vq:%.2fV I:%.2fA accum:%.3f/%.3f trip:%.0f%% Vlim:%.2fV | "
                "M2 %.0fHz Vq:%.2fV I:%.2fA accum:%.3f/%.3f trip:%.0f%% Vlim:%.2fV\n",
                d1.loopFreqHz, d1.voltageQ, d1.currentEst, d1.i2tAccum, d1.i2tThreshold,
                d1.tripRatio * 100.f, d1.voltageLimit,
                d2.loopFreqHz, d2.voltageQ, d2.currentEst, d2.i2tAccum, d2.i2tThreshold,
                d2.tripRatio * 100.f, d2.voltageLimit);
        }
    }
}

// ============================================================================
#pragma endregion

// ============================================================================
#pragma region SETUP
// ============================================================================

// Hardware initialization and task dispatch
void setup()
{
    delay(2000);
    Serial.begin(115200);
    delay(100);

    // I2C
    Wire.begin(I2C_SDA, I2C_SCL);

    // Button
    pinMode(BOOT_BUTTON_PIN, INPUT_PULLUP);

    // Distance Sensor
    distanceSensor.setTimeout(500);
    if (!distanceSensor.init())
    {
        Serial.println("Failed to detect and initialize VL53L1X sensor");
        while (1)
        {
        };
    }

    // LED
    FastLED.addLeds<SK6812, RGB_LED_PIN>(onboard_led, 1);
    FastLED.setBrightness(50);
    onboard_led[0] = CRGB::Black;
    FastLED.show();

    // IMU & Mahony
    if (!imu.begin(Wire, MPU_ADDR,
                   CAL_XA, CAL_YA, CAL_ZA,
                   CAL_XG, CAL_YG, CAL_ZG,
                   SAMPLE_HZ,
                   0.00f)) // yawDecay: 0.05 → ~20 s time constant
    {
        Serial.println("IMU not found — check wiring");
        while (true)
        {
            delay(10);
        }
    }

    // SimpleFOC
    SimpleFOCDebug::enable(&Serial);
    SPI.begin(SENSOR_SPI_CLK, SENSOR_SPI_MISO, SENSOR_SPI_MOSI);

    // Motor 1 Configuration
    EarMotorConfig cfg1;
    cfg1.forwardAngle = 6.1f;
    cfg1.backAngle = 3.6f;
    cfg1.zeroElectricAngle = 4.44;
    cfg1.sensorDirection = Direction::CW;

    // Motor 2 Configuration
    EarMotorConfig cfg2;
    cfg2.forwardAngle = 1.1f;
    cfg2.backAngle = 3.6f;
    cfg2.zeroElectricAngle = 0.57;
    cfg2.sensorDirection = Direction::CW;

    // Motor Initialization
    mot1 = new EarMotor(SPI, B1_SENSOR_CS_PIN,
                        B1_MOTOR_PWM_A, B1_MOTOR_PWM_B, B1_MOTOR_PWM_C, B1_MOTOR_ENABLE, cfg1);
    mot2 = new EarMotor(SPI, B2_SENSOR_CS_PIN,
                        B2_MOTOR_PWM_A, B2_MOTOR_PWM_B, B2_MOTOR_PWM_C, B2_MOTOR_ENABLE, cfg2);

    if (mot1->init() != EarMotor::InitResult::OK)
    {
        Serial.println("Motor 1 init failed");
        while (true)
        {
            delay(10);
        }
    }
    if (mot2->init() != EarMotor::InitResult::OK)
    {
        Serial.println("Motor 2 init failed");
        while (true)
        {
            delay(10);
        }
    }

    // ============================================================================
    // Task Dispatch
    // ============================================================================

    xTaskCreatePinnedToCore(
        FOC_Task,
        "FOC_Task",
        4096,
        NULL,
        configMAX_PRIORITIES - 1,
        NULL,
        !ARDUINO_RUNNING_CORE);
}

// ============================================================================
#pragma endregion

// ============================================================================
#pragma region MAIN LOOP
// ============================================================================

void loop()
{
}

// ============================================================================
#pragma endregion
