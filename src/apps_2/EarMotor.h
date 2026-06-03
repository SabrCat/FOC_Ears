#pragma once
#include <SimpleFOC.h>
#include <encoders/MT6701/MagneticSensorMT6701SSI.h>

struct EarMotorConfig
{
    // Motor electrical
    uint8_t polePairs = 7;
    float phaseResistance = 3.25f;
    float kv = 330.0f;

    // Driver
    float supplyVoltage = 12.0f;
    float voltageLimit = 12.0f;
    uint32_t pwmFrequency = 40000;

    // Physical range in raw sensor radians — REQUIRED, no defaults.
    // Set these to the actual sensor readings at each mechanical limit.
    // setPosition(0) → forwardAngle, setPosition(throw) → backAngle.
    // throw = |backAngle - forwardAngle|, computed automatically.
    // init() returns CONFIG_ERROR if either is left as NAN.
    float forwardAngle = NAN;
    float backAngle = NAN;

    // Stored calibration. If zeroElectricAngle != NOT_SET, init() skips the
    // alignment spin and uses these values directly. Call getCalibration()
    // after first-run init() to retrieve values for persistent storage.
    float zeroElectricAngle = NOT_SET;
    Direction sensorDirection = Direction::UNKNOWN;

    // Angle PID
    float anglePidP = 50.0f;
    float anglePidI = 0.0f;
    float anglePidD = 0.5f;
    float anglePidLimit = 20.0f; // rad/s output cap from angle loop
    float angleLpfTf = 0.01f;

    // Velocity PID
    float velPidP = 0.1f;
    float velPidI = 0.0f;
    float velPidD = 0.0f;
    float velLpfTf = 0.1f;

    // LPF on the position command fed to motor.move() — smooths abrupt target changes
    float commandLpfTf = 0.02f;

    // I²t thermal protection.
    // Voltage ramps down from voltageLimit to continuousVoltage as thermal
    // accumulator fills, recovering automatically when current drops.
    float continuousVoltage = 2.0f; // V — long-run ceiling
    float thermalTimeConst = 20.0f; // s — thermal decay time constant
};

class EarMotor
{
public:
    enum class InitResult
    {
        OK,
        CONFIG_ERROR,
        FOC_FAILED,
        OUT_OF_RANGE
    };

    struct Calibration
    {
        float zeroElectricAngle;
        Direction sensorDirection;
    };

    struct DiagState
    {
        float voltageQ;     // |q-axis voltage| applied last cycle [V]
        float currentEst;   // estimated current: voltageQ / phaseResistance [A]
        float i2tAccum;     // thermal accumulator value [A²·s]
        float i2tThreshold; // trip threshold: I_rated² · τ [A²·s]
        float tripRatio;    // 0 = cool, 1 = fully limited
        float voltageLimit; // effective motor voltage limit after I²t [V]
        float loopFreqHz;   // FOC loop rate, EMA-smoothed [Hz]
    };

    EarMotor(SPIClass &spi, uint8_t sensorCS,
             uint8_t pwmA, uint8_t pwmB, uint8_t pwmC, uint8_t enable,
             const EarMotorConfig &cfg);

    // Call once from setup / FreeRTOS task before entering the update loop.
    // Performs the alignment spin only when no stored calibration is provided.
    InitResult init();

    // Call at full rate from the dedicated FOC task: runs loopFOC + move + I²t.
    void update();

    // Position in radians from the forward limit (0 = forward, throw = back).
    // Safe to call from another core — volatile write, no mutex needed.
    void setPosition(float radFromForward);
    float getPosition() const;

    // Retrieve calibration after first-run init() to persist to NVS.
    Calibration getCalibration() const;

    bool isI2tActive() const { return _i2tActive; }
    const DiagState &diagState() const { return _diag; }

private:
    SPIClass &_spi;
    SPISettings _spiSettings;
    MagneticSensorMT6701SSI _sensor;
    BLDCDriver3PWM _driver;
    BLDCMotor _motor;

    const EarMotorConfig _cfg;
    const float _throw;    // |backAngle - forwardAngle|
    LowPassFilter _cmdLpf; // smooths position commands

    volatile float _targetRad = 0.0f;
    bool _i2tActive = false;
    float _i2tAccum = 0.0f;
    uint32_t _lastUpdateUs = 0;
    DiagState _diag = {};

    float toSensorAngle(float radFromForward) const;
    float fromSensorAngle(float sensorRad) const;
};
