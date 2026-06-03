// ============================================================================
// CLOSED-LOOP VELOCITY & POSITION TEST
// ============================================================================
// Validates closed-loop FOC control using the MT6701 magnetic encoder.
// Supports both velocity and angle (position) control modes.
//
// Serial tuning is handled via the SimpleFOC Commander interface.
// Connect a terminal at 115200 baud and type 'M?' for a full command listing.
//
// Quick-reference shortcuts (single character, no newline required):
//   v - switch to closed-loop VELOCITY mode (target → 0 rad/s)
//   p - switch to closed-loop POSITION mode (target → current angle)
//   a - increase target  (+1 rad/s  or  +0.5 rad)
//   s - decrease target  (-1 rad/s  or  -0.5 rad)
//   r - reset target to 0
//   q - stop motor (zero target)
//
// Commander motor commands (prefix 'M'):
//   MT<value>  – set target          e.g.  MT3.14
//   MC<0-4>    – controller type     1=velocity, 2=angle
//   MVP<value> – velocity P gain
//   MVI<value> – velocity I gain
//   MVD<value> – velocity D gain
//   MAP<value> – angle P gain
//   MLV<value> – velocity LPF Tf
//   MS         – report motor status
// ============================================================================

#include "Arduino.h"
#include "SPI.h"
#include "SimpleFOC.h"
#include "SimpleFOCDrivers.h"
#include "encoders/MT6701/MagneticSensorMT6701SSI.h"

// ============================================================================
// PIN CONFIGURATION - Modify these for your hardware setup
// ============================================================================

// SPI Configuration for Sensor
#define SENSOR_SPI_CLK 9
#define SENSOR_SPI_MISO 10
#define SENSOR_SPI_MOSI 11 // MOSI is not used but must be specified

// Board 1
// #define SENSOR_CS_PIN 12
// #define MOTOR_PWM_A 13
// #define MOTOR_PWM_B 14
// #define MOTOR_PWM_C 15
// #define MOTOR_ENABLE 16

// Board 2 (uncomment and comment Board 1 above to use)
#define SENSOR_CS_PIN 48
#define MOTOR_PWM_A 26
#define MOTOR_PWM_B 47
#define MOTOR_PWM_C 33
#define MOTOR_ENABLE 34

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

MagneticSensorMT6701SSI sensor = MagneticSensorMT6701SSI(SENSOR_CS_PIN, spiSettings);
BLDCDriver3PWM driver = BLDCDriver3PWM(MOTOR_PWM_A, MOTOR_PWM_B, MOTOR_PWM_C, MOTOR_ENABLE);
BLDCMotor motor = BLDCMotor(MOTOR_POLE_PAIRS, MOTOR_PHASE_RESISTANCE, MOTOR_KV);

Commander commander = Commander(Serial); //, '\n', false);
void onMotor(char *cmd) { commander.motor(&motor, cmd); }

// ============================================================================
// SETUP
// ============================================================================

void setup()
{
    delay(2000);
    Serial.begin(115200);
    delay(100);

    // Serial.println("\n=== FOC Ears Closed-Loop Test ===");

    SimpleFOCDebug::enable(&Serial);

    // SPI + sensor
    SPI.begin(SENSOR_SPI_CLK, SENSOR_SPI_MISO, SENSOR_SPI_MOSI);
    sensor.init();
    // Serial.println("Sensor initialised");

    // Driver
    // driver.pwm_frequency = PWM_FREQUENCY;
    driver.voltage_power_supply = VOLTAGE_POWER_SUPPLY;
    // driver.voltage_limit = VOLTAGE_POWER_SUPPLY;
    driver.init();
    // Serial.println("Driver initialised");

    // Motor
    motor.linkSensor(&sensor);
    motor.linkDriver(&driver);

    // Start in closed-loop velocity mode
    motor.controller = MotionControlType::velocity;

    motor.PID_velocity.P = 0.6;
    motor.PID_velocity.I = 0;
    motor.PID_velocity.D = 0; // motor gets very noisy if this is on
    motor.PID_velocity.output_ramp = NOT_SET;
    motor.PID_velocity.limit = driver.voltage_limit;
    motor.LPF_velocity.Tf = 0.1; // needed to stabilize high P value

    motor.P_angle.P = 50;
    motor.P_angle.I = 0;
    motor.P_angle.D = 0.5; // Angle D term works very well, but noisy without LPF
    motor.P_angle.output_ramp = NOT_SET;
    motor.P_angle.limit = 20;
    motor.LPF_angle.Tf = 0.005;

    motor.voltage_limit = VOLTAGE_POWER_SUPPLY;

    motor.useMonitoring(Serial);
    motor.monitor_downsample = 0;
    commander.verbose = VerboseMode::machine_readable;

    motor.init();

    motor.initFOC();

    // Register Commander
    commander.add('M', onMotor, "motor");
}

// ============================================================================
// LOOP
// ============================================================================

float target = 0.0f;
unsigned long lastPrint = 0;

void loop()
{
    motor.loopFOC();
    motor.move(motor.target);
    motor.monitor();
    commander.run();
}
