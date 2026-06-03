#pragma once
#include "MahonyAHRS.h"

class IMUHandler
{
private:
    Mahony filter;

    float ax;
    float ay;
    float az;

    float gx;
    float gy;
    float gz;

    float gyroBiasX = 0.0f;
    float gyroBiasY = 0.0f;
    float gyroBiasZ = 0.0f;

    void writeRegister();
    void readBytes();
    void readMPU6050();
    void setupMPU6050();
    void calibrateGyros();

public:
    IMUHandler(/* args */);
    ~IMUHandler();
};

IMUHandler::IMUHandler(/* args */)
{
}

IMUHandler::~IMUHandler()
{
}
