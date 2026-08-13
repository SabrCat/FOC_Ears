#pragma once
#include "../BoardProfile.h"

// Board 2 — no distance sensor populated. Selected by -D FOCEARS_BOARD=2.
//
// Fill the TODO fields by running the `board_calibration` environment on this
// physical unit; it prints a paste-ready block. Motor limits are NAN until then:
// EarMotor::init() returns CONFIG_ERROR (LED solid red) so an un-provisioned board
// fails loudly instead of moving to a wrong position.
static constexpr BoardProfile ACTIVE_BOARD = {
    .name = "board-2",

    .supplyVoltage = 12.0f,
    .voltageLimit = 12.0f,

    // IMU offsets: fill from the mpu6050_calibration_test environment
    .imuAccelOffsetX = -1883,
    .imuAccelOffsetY = -1169,
    .imuAccelOffsetZ = 857,
    .imuGyroOffsetX = 50,
    .imuGyroOffsetY = -4,
    .imuGyroOffsetZ = -6,

    .motor1ForwardLimit = 5.3287f,
    .motor1BackLimit = 2.8287f,
    .motor1ZeroElectricAngle = 3.6597f,
    .motor1Direction = Direction::CW,

    .motor2ForwardLimit = 4.5145f,
    .motor2BackLimit = 7.0145f,
    .motor2ZeroElectricAngle = 1.7085f,
    .motor2Direction = Direction::CW,

    .expectsDistanceSensor = false,
};
