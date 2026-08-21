#pragma once
#include <SimpleFOC.h> // Direction, NOT_SET

// Per-physical-unit configuration.
//
// Every field here is specific to ONE assembled board and must be measured on
// that unit — nothing here is shared between boards. The set of board files in
// boards/ is the git-tracked source of truth: if a unit's flash is wiped, the
// firmware is recoverable by reflashing its profile. Do not copy calibration
// values between boards; capture them per unit.
//
// Provisioning a new board (see boards/board2_collaborator.h for a template):
//   - IMU offsets       : run the `mpu6050_calibration_test` environment
//   - motorN limits     : hold each ear to its stop, read raw angle via `sensor_test`
//   - motorN elec/dir   : leave NOT_SET / UNKNOWN → first boot runs the alignment
//                         spin and prints the values (SimpleFOCDebug), then paste
//                         them back in to skip the spin on subsequent boots.
struct BoardProfile
{
    const char *name; // human label, printed at boot: "board-1"

    // Driver supply — USB-C PD voltage set by this unit's solder jumpers — and the
    // FOC control-voltage cap (≤ supply; lower than supply to limit torque/heat).
    // Both are per-unit: supply is a hardware fact, limit a per-unit tuning choice.
    float supplyVoltage;
    float voltageLimit;

    // IMU calibration offsets — from mpu6050_calibration_test "Active offsets".
    int16_t imuAccelOffsetX, imuAccelOffsetY, imuAccelOffsetZ;
    int16_t imuGyroOffsetX, imuGyroOffsetY, imuGyroOffsetZ;

    // Motor 1 — right ear (driver board 1).
    float motor1ForwardLimit;      // raw sensor rad at the forward mechanical stop
    float motor1BackLimit;         // raw sensor rad at the back mechanical stop
    float motor1ZeroElectricAngle; // NOT_SET → alignment spin runs on boot
    Direction motor1Direction;     // Direction::UNKNOWN → determined by the spin

    // Motor 2 — left ear (driver board 2).
    float motor2ForwardLimit;
    float motor2BackLimit;
    float motor2ZeroElectricAngle;
    Direction motor2Direction;

    // True if this unit's ears have hard mechanical stops at their travel limits.
    // Gates the OUT_OF_RANGE boot check per unit (see EarMotorConfig). A unit without
    // stops rests at any angle and must not brick on an off-arc boot. Applies to both
    // ears of the unit; split into per-motor flags if one ear ever differs.
    bool hasMechanicalStops;

    // Control-loop tuning — one set applied to BOTH ears of this unit, because
    // mechanics and load are per-unit. Defaults match EarMotorConfig; override per
    // unit as tuning requires. Split into per-motor sets if the two ears diverge.
    float anglePidP, anglePidI, anglePidD;
    float anglePidLimit; // rad/s output cap from the angle loop
    float angleLpfTf;
    float velPidP, velPidI, velPidD;
    float velLpfTf;

    // True if this unit is populated with a VL53L1X distance sensor. When true
    // and the sensor fails to init, the firmware degrades (no headpat) and warns
    // via the LED rather than bricking. When false, the sensor is skipped silently.
    bool expectsDistanceSensor;
};
