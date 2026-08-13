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

    .expectsDistanceSensor = true,
};
