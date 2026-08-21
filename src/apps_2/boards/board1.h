#pragma once
#include "../BoardProfile.h"

// Board 1 — the original prototype unit. Values captured by hand off serial
// during bring-up. Selected by -D FOCEARS_BOARD=1.
static constexpr BoardProfile ACTIVE_BOARD = {
    .name = "board-1",

    .supplyVoltage = 12.0f,
    .voltageLimit = 12.0f,

    .imuAccelOffsetX = -3181,
    .imuAccelOffsetY = -761,
    .imuAccelOffsetZ = 455,
    .imuGyroOffsetX = 54,
    .imuGyroOffsetY = -7,
    .imuGyroOffsetZ = 4,

    .motor1ForwardLimit = 6.1f,
    .motor1BackLimit = 3.6f,
    .motor1ZeroElectricAngle = 4.44f,
    .motor1Direction = Direction::CW,

    .motor2ForwardLimit = 1.1f,
    .motor2BackLimit = 3.6f,
    .motor2ZeroElectricAngle = 0.57f,
    .motor2Direction = Direction::CW,

    .hasMechanicalStops = true, // original prototype — both ears have hard stops

    // Control-loop tuning — current defaults (unchanged from EarMotorConfig).
    .anglePidP = 20.0f,
    .anglePidI = 0.0f,
    .anglePidD = 0.1f,
    .anglePidLimit = 20.0f,
    .angleLpfTf = 0.01f,
    .velPidP = 0.07f,
    .velPidI = 0.0f,
    .velPidD = 0.0f,
    .velLpfTf = 0.1f,

    .expectsDistanceSensor = true,
};
