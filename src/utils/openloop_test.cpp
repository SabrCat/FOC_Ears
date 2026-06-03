// ============================================================================
// OPENLOOP VELOCITY TEST
// ============================================================================
// Quick hardware validation test for sensor and motor driver
// - Brings up a single MT6701 sensor via SPI
// - Brings up a BLDC motor driver on 3-PWM mode
// - Runs open-loop velocity control to test basic functionality
// 
// Pin Configuration
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
#define SENSOR_SPI_CLK              9        // SPI clock (MOSI not used but required)
#define SENSOR_SPI_MISO            10        // SPI MISO (data out)
#define SENSOR_SPI_MOSI            11        // SPI MOSI (not used, set to -1)

// Motor Driver - 3 PWM + Enable Configuration

// Board 1
#define SENSOR_CS_PIN              12      // MT6701 chip select
#define MOTOR_PWM_A                13        // Phase A PWM
#define MOTOR_PWM_B                14        // Phase B PWM
#define MOTOR_PWM_C                15        // Phase C PWM
#define MOTOR_ENABLE               16        // Driver enable pin

// Board 2
// #define SENSOR_CS_PIN              48      // MT6701 chip select
// #define MOTOR_PWM_A                26        // Phase A PWM
// #define MOTOR_PWM_B                47        // Phase B PWM
// #define MOTOR_PWM_C                33        // Phase C PWM
// #define MOTOR_ENABLE               34        // Driver enable pin

// Motor Electrical Configuration
#define MOTOR_POLE_PAIRS            7        // Mitoot 2804 motor: 7 pole pairs
#define MOTOR_PHASE_RESISTANCE      3.25f    // Ohms (6.5 / 2)
#define MOTOR_KV                  330       // RPM per Volt

// Controller Configuration
#define VOLTAGE_POWER_SUPPLY        9.0f     // Supply voltage in Volts
#define PWM_FREQUENCY           40000     // PWM frequency in Hz
#define TARGET_VELOCITY             1      // rad/s for open-loop test

// ============================================================================
// HARDWARE OBJECTS
// ============================================================================

// Configure SPI settings for MT6701
static SPISettings spiSettings(4000000, MT6701_BITORDER, SPI_MODE2);

// Create sensor instance
MagneticSensorMT6701SSI sensor = MagneticSensorMT6701SSI(SENSOR_CS_PIN, spiSettings);

// Create motor driver instance
BLDCDriver3PWM driver = BLDCDriver3PWM(MOTOR_PWM_A, MOTOR_PWM_B, MOTOR_PWM_C, MOTOR_ENABLE);

// Create motor instance
BLDCMotor motor = BLDCMotor(MOTOR_POLE_PAIRS, MOTOR_PHASE_RESISTANCE, MOTOR_KV);

// ============================================================================
// SETUP
// ============================================================================
void setup()
{
  delay(2000);

  // Initialize serial communication for debugging
  Serial.begin(115200);
  delay(100);
  
  Serial.println("\n=== FOC Ears OpenLoop Test ===");
  Serial.println("Initializing sensor...");

  // Initialize SPI bus
  SPI.begin(SENSOR_SPI_CLK, SENSOR_SPI_MISO, SENSOR_SPI_MOSI);

  // Initialize sensor
  sensor.init();

  // Configure driver
  Serial.println("Initializing motor driver...");
  driver.pwm_frequency = PWM_FREQUENCY;
  driver.voltage_power_supply = VOLTAGE_POWER_SUPPLY;
  driver.voltage_limit = VOLTAGE_POWER_SUPPLY;
  driver.init();
  Serial.println("Motor driver initialized");

  // Configure motor with open-loop velocity control
  Serial.println("Configuring motor for open-loop velocity control...");
  motor.linkDriver(&driver);
  motor.linkSensor(&sensor);
  motor.controller = MotionControlType::velocity_openloop;
  motor.init();
  Serial.println("Motor configured");

  Serial.print("Starting open-loop velocity control at ");
  Serial.print(TARGET_VELOCITY);
  Serial.println(" rad/s");
  Serial.println("Commands:");
  Serial.println("  'a' - adjust velocity up");
  Serial.println("  's' - adjust velocity down");
  Serial.println("  'r' - reset velocity");
  Serial.println("  'q' - stop motor");
  Serial.println("");
}

// ============================================================================
// LOOP
// ============================================================================

float targetVelocity = TARGET_VELOCITY;
unsigned long lastPrint = 0;

void loop()
{
  // Handle serial commands for real-time velocity adjustment
  if (Serial.available() > 0) {
    char command = Serial.read();
    switch (command) {
      case 'a':  // Increase velocity
        targetVelocity += 2.0f;
        Serial.print("Target velocity: ");
        Serial.print(targetVelocity);
        Serial.println(" rad/s");
        break;
      case 's':  // Decrease velocity
        targetVelocity -= 2.0f;
        Serial.print("Target velocity: ");
        Serial.print(targetVelocity);
        Serial.println(" rad/s");
        break;
      case 'r':  // Reset to default
        targetVelocity = TARGET_VELOCITY;
        Serial.print("Reset to: ");
        Serial.print(targetVelocity);
        Serial.println(" rad/s");
        break;
      case 'q':  // Stop motor
        targetVelocity = 0;
        Serial.println("Motor stopped");
        break;
    }
  }

  // Run motor control loop
  motor.loopFOC();
  motor.move(targetVelocity);

  // Print status periodically
  if (millis() - lastPrint > 100) {
    lastPrint = millis();
    
    Serial.print("Sensor: ");
    Serial.print(sensor.getAngle(), 3);
    Serial.print(" rad | Velocity: ");
    Serial.print(motor.shaft_velocity, 3);
    Serial.print(" rad/s | Target: ");
    Serial.print(targetVelocity, 3);
    Serial.println(" rad/s");
  }
}