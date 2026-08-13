// ============================================================================
// BOARD CALIBRATION / PROVISIONING
// ============================================================================
// One-shot per-unit provisioning. Runs the FOC alignment on both motors to find
// their electric zero + sensor direction, guides you through capturing each ear's
// FORWARD stop, autodetects the distance sensor, and prints a paste-ready
// BoardProfile block for src/apps_2/boards/boardN.h.
//
// The BACK limit is COMPUTED (forward + sign*EAR_THROW), not measured: the back
// stop is mushy, and ear-to-ear consistency matters more than hitting it exactly.
// The per-motor sign is the fixed motor-mounting handedness — it never changes.
//
// IMU offsets are NOT covered here (that calibration is slow by design); run the
// `mpu6050_calibration_test` environment separately and paste those in too.
//
// Usage: flash this env, open a terminal at 115200 baud, follow the prompts.
// ============================================================================

#include "Arduino.h"
#include "SPI.h"
#include <Wire.h>
#include "SimpleFOC.h"
#include "SimpleFOCDrivers.h"
#include "encoders/MT6701/MagneticSensorMT6701SSI.h"
#include <VL53L1X.h>

// ── Pins (must match the hardware / foc_ears_main.cpp) ──────────────────────
#define I2C_SDA 17
#define I2C_SCL 18
#define SENSOR_SPI_CLK 9
#define SENSOR_SPI_MISO 10
#define SENSOR_SPI_MOSI 11
// Driver board 1
#define B1_SENSOR_CS_PIN 12
#define B1_MOTOR_PWM_A 13
#define B1_MOTOR_PWM_B 14
#define B1_MOTOR_PWM_C 15
#define B1_MOTOR_ENABLE 16
// Driver board 2
#define B2_SENSOR_CS_PIN 48
#define B2_MOTOR_PWM_A 26
#define B2_MOTOR_PWM_B 47
#define B2_MOTOR_PWM_C 33
#define B2_MOTOR_ENABLE 34

// ── Motor electrical ────────────────────────────────────────────────────────
#define MOTOR_POLE_PAIRS 7
#define MOTOR_PHASE_RESISTANCE 3.25f
#define MOTOR_KV 330

// Ear mechanical throw — fixed by the ear/stop geometry, shared across all units.
// Must match MOTOR_THROW in foc_ears_main.cpp's Anim_Task.
static constexpr float EAR_THROW = 2.5f;

static SPISettings spiSettings(4000000, MT6701_BITORDER, SPI_MODE2);

// SimpleFOC objects declared as globals (house style — not copyable, so the rig
// below references them rather than owning them).
MagneticSensorMT6701SSI sensor1(B1_SENSOR_CS_PIN, spiSettings);
BLDCDriver3PWM driver1(B1_MOTOR_PWM_A, B1_MOTOR_PWM_B, B1_MOTOR_PWM_C, B1_MOTOR_ENABLE);
BLDCMotor motor1(MOTOR_POLE_PAIRS, MOTOR_PHASE_RESISTANCE, MOTOR_KV);

MagneticSensorMT6701SSI sensor2(B2_SENSOR_CS_PIN, spiSettings);
BLDCDriver3PWM driver2(B2_MOTOR_PWM_A, B2_MOTOR_PWM_B, B2_MOTOR_PWM_C, B2_MOTOR_ENABLE);
BLDCMotor motor2(MOTOR_POLE_PAIRS, MOTOR_PHASE_RESISTANCE, MOTOR_KV);

// One motor's hardware + the fixed handedness of its back limit.
struct MotorRig
{
    MagneticSensorMT6701SSI &sensor;
    BLDCDriver3PWM &driver;
    BLDCMotor &motor;
    const char *label;
    float backSign; // back = forward + backSign*EAR_THROW (fixed motor mounting)
    float forwardLimit;
    float backLimit;
    bool ppCheckOk;
};

// backSign fixed per motor: board-1 has motor1 forward 6.1 -> back 3.6 (-), motor2 1.1 -> 3.6 (+).
MotorRig rigs[2] = {
    {sensor1, driver1, motor1, "motor1 (right ear)", -1.0f, NAN, NAN, false},
    {sensor2, driver2, motor2, "motor2 (left ear)", +1.0f, NAN, NAN, false},
};

// ── Serial helpers (blocking — this is an interactive bring-up sketch) ───────
static float promptFloat(const char *msg)
{
    Serial.print(msg);
    while (Serial.available())
        Serial.read(); // flush stale input
    while (!Serial.available())
        delay(5);
    float v = Serial.parseFloat();
    while (Serial.available())
        Serial.read(); // drain rest of line
    Serial.println(v);
    return v;
}

static void waitForEnter(const char *msg)
{
    Serial.print(msg);
    while (Serial.available())
        Serial.read();
    while (!Serial.available())
        delay(5);
    while (Serial.available())
        Serial.read();
    Serial.println(" [captured]");
}

static float readMechAngle(MagneticSensorMT6701SSI &s)
{
    s.update();
    return s.getMechanicalAngle(); // [0, 2π), matches the stored-limit convention
}

static const char *dirStr(Direction d)
{
    return d == Direction::CW ? "CW" : d == Direction::CCW ? "CCW"
                                                           : "UNKNOWN";
}

// ── Calibration flow ────────────────────────────────────────────────────────
void setup()
{
    delay(2000);
    Serial.begin(115200);
    delay(100);
    Serial.setTimeout(5000);

    Serial.println("\n=== FOC Ears -- Board Calibration ===");
    SimpleFOCDebug::enable(&Serial);

    Wire.begin(I2C_SDA, I2C_SCL);
    SPI.begin(SENSOR_SPI_CLK, SENSOR_SPI_MISO, SENSOR_SPI_MOSI);

    const float supply = promptFloat("Supply voltage (match the PD solder jumpers), e.g. 12: ");
    const float vlimit = fminf(12.0f, supply); // default cap at 12 V for torque/heat

    // Align both motors — energizes briefly, then disables so the ear moves freely.
    for (MotorRig &rig : rigs)
    {
        rig.sensor.init(&SPI);
        rig.driver.voltage_power_supply = supply;
        rig.driver.voltage_limit = supply;
        rig.driver.pwm_frequency = 40000;
        rig.driver.init();
        rig.motor.linkSensor(&rig.sensor);
        rig.motor.linkDriver(&rig.driver);
        rig.motor.controller = MotionControlType::angle;
        rig.motor.voltage_limit = vlimit;
        rig.motor.init();
        rig.motor.initFOC();
        rig.ppCheckOk = rig.motor.pp_check_result; // valid: we align with sensor_direction UNKNOWN
        Serial.printf("%s aligned: zero_electric_angle=%.4f  sensor_direction=%s  PP_check=%s\n",
                      rig.label, rig.motor.zero_electric_angle, dirStr(rig.motor.sensor_direction),
                      rig.ppCheckOk ? "OK" : "FAIL");
        rig.motor.disable(); // free the ear for manual positioning
    }

    // Distance sensor autodetect.
    VL53L1X tof;
    tof.setTimeout(500);
    const bool tofPresent = tof.init();
    Serial.printf("Distance sensor (VL53L1X): %s\n", tofPresent ? "DETECTED" : "not found");

    // Capture each ear's forward stop; compute the back limit from the fixed throw.
    // Loud, non-scrolling warning if either pole-pair check failed. A failed PP check
    // means the alignment (and its zero_electric_angle) is unreliable, usually from
    // mechanical friction/binding during the spin. Gate on a keypress so it can't scroll.
    if (!rigs[0].ppCheckOk || !rigs[1].ppCheckOk)
    {
        Serial.println("\n########################################################");
        Serial.println("##  WARNING: POLE-PAIR CHECK FAILED                   ##");
        Serial.printf("##  motor1 PP: %-4s   motor2 PP: %-4s\n",
                      rigs[0].ppCheckOk ? "OK" : "FAIL", rigs[1].ppCheckOk ? "OK" : "FAIL");
        Serial.println("##  Alignment / zero angle is UNRELIABLE. Usual cause: ##");
        Serial.println("##  mechanical friction or binding during the spin.    ##");
        Serial.println("##  Fix it and re-run before trusting these values.    ##");
        Serial.println("########################################################");
        waitForEnter("Press Enter to acknowledge and continue anyway:");
    }

    Serial.println("\nMotors are now de-energized. Position each ear by hand.");
    for (MotorRig &rig : rigs)
    {
        char msg[96];
        snprintf(msg, sizeof(msg), "Hold %s against its FORWARD stop, then press Enter:", rig.label);
        waitForEnter(msg);
        // Forward is normalized to [0, 2π) so twisting the motor through full turns
        // while positioning can't corrupt it. Back stays UNWRAPPED (may exceed 2π) so
        // |back-forward| == throw for the linear interp; EarMotor::init's wrap-safe
        // re-anchor handles limits outside [0, 2π).
        rig.forwardLimit = _normalizeAngle(readMechAngle(rig.sensor));
        rig.backLimit = rig.forwardLimit + rig.backSign * EAR_THROW;
        Serial.printf("  forward=%.4f  ->  back=%.4f (computed)\n", rig.forwardLimit, rig.backLimit);
    }

    // Paste-ready profile block.
    Serial.println("\n============================================================");
    Serial.println("Paste into src/apps_2/boards/boardN.h (set N and the name):");
    Serial.println("============================================================");
    Serial.printf(
        "static constexpr BoardProfile ACTIVE_BOARD = {\n"
        "    .name = \"board-N\",\n\n"
        "    .supplyVoltage = %.1ff,\n"
        "    .voltageLimit = %.1ff,\n\n"
        "    // IMU offsets: fill from the mpu6050_calibration_test environment\n"
        "    .imuAccelOffsetX = 0, .imuAccelOffsetY = 0, .imuAccelOffsetZ = 0,\n"
        "    .imuGyroOffsetX = 0, .imuGyroOffsetY = 0, .imuGyroOffsetZ = 0,\n\n"
        "    .motor1ForwardLimit = %.4ff,\n"
        "    .motor1BackLimit = %.4ff,\n"
        "    .motor1ZeroElectricAngle = %.4ff,\n"
        "    .motor1Direction = Direction::%s,\n\n"
        "    .motor2ForwardLimit = %.4ff,\n"
        "    .motor2BackLimit = %.4ff,\n"
        "    .motor2ZeroElectricAngle = %.4ff,\n"
        "    .motor2Direction = Direction::%s,\n\n"
        "    .expectsDistanceSensor = %s,\n"
        "};\n",
        supply, vlimit,
        rigs[0].forwardLimit, rigs[0].backLimit, rigs[0].motor.zero_electric_angle, dirStr(rigs[0].motor.sensor_direction),
        rigs[1].forwardLimit, rigs[1].backLimit, rigs[1].motor.zero_electric_angle, dirStr(rigs[1].motor.sensor_direction),
        tofPresent ? "true" : "false");
    Serial.println("============================================================");
    Serial.println("Done. Reset the board to re-run.");
}

void loop()
{
}
