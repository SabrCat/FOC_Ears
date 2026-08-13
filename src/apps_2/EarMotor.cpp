#include "EarMotor.h"
#include <math.h>

namespace
{
// Map an angle to (-π, π]. Used to unwrap the boot position onto the travel arc.
inline float wrapToPi(float x)
{
    return x - _2PI * floorf(x / _2PI + 0.5f);
}
} // namespace

EarMotor::EarMotor(SPIClass &spi, uint8_t sensorCS,
                   uint8_t pwmA, uint8_t pwmB, uint8_t pwmC, uint8_t enable,
                   const EarMotorConfig &cfg)
    : _spi(spi),
      _spiSettings(4000000, MT6701_BITORDER, SPI_MODE2),
      _sensor(sensorCS, _spiSettings),
      _driver(pwmA, pwmB, pwmC, enable),
      _motor(cfg.polePairs, cfg.phaseResistance, cfg.kv),
      _cfg(cfg),
      _throw(fabsf(cfg.backAngle - cfg.forwardAngle)),
      _cmdLpf(cfg.commandLpfTf)
{
}

EarMotor::InitResult EarMotor::init()
{
    if (isnan(_cfg.forwardAngle) || isnan(_cfg.backAngle))
        return InitResult::CONFIG_ERROR;

    _sensor.init(&_spi);

    _driver.voltage_power_supply = _cfg.supplyVoltage;
    _driver.voltage_limit = _cfg.voltageLimit;
    _driver.pwm_frequency = _cfg.pwmFrequency;
    _driver.init();

    _motor.linkSensor(&_sensor);
    _motor.linkDriver(&_driver);

    _motor.controller = MotionControlType::angle;

    _motor.PID_velocity.P = _cfg.velPidP;
    _motor.PID_velocity.I = _cfg.velPidI;
    _motor.PID_velocity.D = _cfg.velPidD;
    _motor.PID_velocity.output_ramp = NOT_SET;
    _motor.LPF_velocity.Tf = _cfg.velLpfTf;

    _motor.P_angle.P = _cfg.anglePidP;
    _motor.P_angle.I = _cfg.anglePidI;
    _motor.P_angle.D = _cfg.anglePidD;
    _motor.P_angle.output_ramp = NOT_SET;
    _motor.P_angle.limit = _cfg.anglePidLimit;
    _motor.LPF_angle.Tf = _cfg.angleLpfTf;

    _motor.voltage_limit = _cfg.voltageLimit;

    _motor.init();

    // Set calibration after init() so it cannot be reset by it.
    // initFOC() skips the alignment sweep only when BOTH fields are provided.
    if (_cfg.zeroElectricAngle != NOT_SET && _cfg.sensorDirection != Direction::UNKNOWN)
    {
        _motor.zero_electric_angle = _cfg.zeroElectricAngle;
        _motor.sensor_direction = _cfg.sensorDirection;
    }

    if (!_motor.initFOC())
        return InitResult::FOC_FAILED;

    // Ensure voltage_limit is at the configured max after initFOC (which may
    // have temporarily drawn current during the alignment spin).
    _motor.voltage_limit = _cfg.voltageLimit;

    // Wrap-safe branch re-anchor. The angle controller drives (target - shaft_angle)
    // with no wrapping, so if the ear boots on a sensor branch ~2π away from the
    // target's branch (encoder seam inside the travel arc, or the ear resting far
    // from forward) it would chase the setpoint the long way — into a mechanical
    // stop. Re-seat shaft_angle onto the arc by shifting sensor_offset a whole number
    // of turns. Multiples of 2π leave every target's physical meaning unchanged and
    // feed only shaftAngle(), not commutation. Well-defined because the throw (< π)
    // is confined by the stops, so the boot position is always within π of the arc
    // midpoint. Assumes sensor_direction CW, as toSensorAngle's raw convention does.
    _sensor.update();
    const float mid = 0.5f * (_cfg.forwardAngle + _cfg.backAngle);
    const float u0 = mid + wrapToPi(_sensor.getMechanicalAngle() - mid); // unwrapped onto arc
    _motor.sensor_offset = _sensor.getAngle() - u0;                      // always k·2π

    // Sanity-check the unwrapped boot position is within the physical range.
    // OUT_OF_RANGE most likely indicates wrong calibration or a disassembled ear.
    const float lo = fminf(_cfg.forwardAngle, _cfg.backAngle);
    const float hi = fmaxf(_cfg.forwardAngle, _cfg.backAngle);
    if (u0 < lo - 0.3f || u0 > hi + 0.3f)
        return InitResult::OUT_OF_RANGE;

    _lastUpdateUs = micros();
    return InitResult::OK;
}

void EarMotor::update()
{
    uint32_t now = micros();
    float dt = static_cast<float>(now - _lastUpdateUs) * 1e-6f;
    _lastUpdateUs = now;
    if (dt <= 0.0f || dt > 0.1f)
        dt = 0.005f; // clamp on first call / task stall

    // I²t — estimate current from q-axis voltage of the previous cycle.
    // motor.voltage.q is what was applied by the last loopFOC(); using it here
    // means the thermal model reflects actual applied power, not commanded.
    float I_est = fabsf(_motor.voltage.q) / _cfg.phaseResistance;
    float I_rated = _cfg.continuousVoltage / _cfg.phaseResistance;
    float decay = expf(-dt / _cfg.thermalTimeConst);
    _i2tAccum = _i2tAccum * decay + I_est * I_est * dt;
    float threshold = I_rated * I_rated * _cfg.thermalTimeConst;
    float tripRatio = constrain(_i2tAccum / threshold, 0.0f, 1.0f);

    // Proportional voltage reduction: full voltageLimit at 0% load, down to
    // continuousVoltage at 100% thermal load. Recovers automatically.
    _motor.voltage_limit = _cfg.continuousVoltage + (_cfg.voltageLimit - _cfg.continuousVoltage) * (1.0f - tripRatio);
    _i2tActive = tripRatio > 0.01f;

    // EMA-smoothed loop frequency (α = 0.05 → ~20-sample window)
    _diag.loopFreqHz = _diag.loopFreqHz * 0.95f + (1.0f / dt) * 0.05f;

    _diag.voltageQ = fabsf(_motor.voltage.q);
    _diag.currentEst = I_est;
    _diag.i2tAccum = _i2tAccum;
    _diag.i2tThreshold = threshold;
    _diag.tripRatio = tripRatio;
    _diag.voltageLimit = _motor.voltage_limit;

    _motor.loopFOC();
    _motor.move(toSensorAngle(_cmdLpf(_targetRad)));
}

void EarMotor::setPosition(float radFromForward)
{
    _targetRad = constrain(radFromForward, 0.0f, _throw);
}

float EarMotor::getPosition() const
{
    return fromSensorAngle(_motor.shaft_angle);
}

EarMotor::Calibration EarMotor::getCalibration() const
{
    return {_motor.zero_electric_angle, _motor.sensor_direction};
}

float EarMotor::toSensorAngle(float radFromForward) const
{
    // Linear interpolation: 0 → forwardAngle, _throw → backAngle.
    // Works for both motor orientations (forward < back or forward > back).
    float t = radFromForward / _throw;
    return _cfg.forwardAngle + t * (_cfg.backAngle - _cfg.forwardAngle);
}

float EarMotor::fromSensorAngle(float sensorRad) const
{
    return (sensorRad - _cfg.forwardAngle) / (_cfg.backAngle - _cfg.forwardAngle) * _throw;
}
