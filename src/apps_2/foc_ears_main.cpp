#include "Arduino.h"
#include "SPI.h"
#include "SimpleFOC.h"
#include "SimpleFOCDrivers.h"
#include "encoders/MT6701/MagneticSensorMT6701SSI.h"
#include <FastLED.h>
#include <VL53L1X.h>

// ============================================================================
// PIN CONFIGURATION
// ============================================================================

// I2C
#define I2C_SDA 17
#define I2C_SCL 18

// SPI Configuration for Angle Sensors
#define SENSOR_SPI_CLK 9
#define SENSOR_SPI_MISO 10
#define SENSOR_SPI_MOSI 11 // MOSI is not used but must be specified

// Board 1
#define B1_SENSOR_CS_PIN 12
#define B1_MOTOR_PWM_A 13
#define B1_MOTOR_PWM_B 14
#define B1_MOTOR_PWM_C 15
#define B1_MOTOR_ENABLE 16

// Board 2
#define B2_SENSOR_CS_PIN 48
#define B2_MOTOR_PWM_A 26
#define B2_MOTOR_PWM_B 47
#define B2_MOTOR_PWM_C 33
#define B2_MOTOR_ENABLE 34

// Button
#define BOOT_BUTTON_PIN 0

// Onboard LED
#define RGB_LED_PIN 8

// ============================================================================
// MOTOR ELECTRICAL CONFIGURATION
// ============================================================================
#define MOTOR_POLE_PAIRS 7           // Mitoot 2804: 7 pole pairs
#define MOTOR_PHASE_RESISTANCE 3.25f // Ohms (6.5 / 2)
#define MOTOR_KV 330                 // RPM/V

// ============================================================================
// DRIVER / CONTROLLER CONFIGURATION
// ============================================================================
#define VOLTAGE_POWER_SUPPLY 12.0f
#define PWM_FREQUENCY 40000

// ============================================================================
// HARDWARE OBJECTS
// ============================================================================

static SPISettings spiSettings(4000000, MT6701_BITORDER, SPI_MODE2);

MagneticSensorMT6701SSI B1_sensor = MagneticSensorMT6701SSI(B1_SENSOR_CS_PIN, spiSettings);
BLDCDriver3PWM B1_driver = BLDCDriver3PWM(B1_MOTOR_PWM_A, B1_MOTOR_PWM_B, B1_MOTOR_PWM_C, B1_MOTOR_ENABLE);
BLDCMotor B1_motor = BLDCMotor(MOTOR_POLE_PAIRS, MOTOR_PHASE_RESISTANCE, MOTOR_KV);

VL53L1X distanceSensor;

CRGB onboard_led[1];

void setup()
{
    delay(2000);
    Serial.begin(115200);
    delay(100);

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

        // FOC

    SimpleFOCDebug::enable(&Serial);

    SPI.begin(SENSOR_SPI_CLK, SENSOR_SPI_MISO, SENSOR_SPI_MOSI);
    B1_sensor.init();

    B1_driver.voltage_power_supply = VOLTAGE_POWER_SUPPLY;
    B1_driver.voltage_limit = VOLTAGE_POWER_SUPPLY;
    B1_driver.init();

    B1_motor.linkSensor(&B1_sensor);
    B1_motor.linkDriver(&B1_driver);

    // Start in closed-loop velocity mode
    B1_motor.controller = MotionControlType::velocity;

    B1_motor.PID_velocity.P = 0.6;
    B1_motor.PID_velocity.I = 0;
    B1_motor.PID_velocity.D = 0; // motor gets very noisy if this is on
    B1_motor.PID_velocity.output_ramp = NOT_SET;
    B1_motor.PID_velocity.limit = B1_driver.voltage_limit;
    B1_motor.LPF_velocity.Tf = 0.1; // needed to stabilize high P value

    B1_motor.P_angle.P = 50;
    B1_motor.P_angle.I = 0;
    B1_motor.P_angle.D = 0.5; // Angle D term works very well, but noisy without LPF
    B1_motor.P_angle.output_ramp = NOT_SET;
    B1_motor.P_angle.limit = 20;
    B1_motor.LPF_angle.Tf = 0.005;

    B1_motor.init();

    B1_motor.initFOC();
}

void loop()
{
    if (digitalRead(BOOT_BUTTON_PIN) == LOW) // Button is pressed (active LOW)
    {
    }
}
