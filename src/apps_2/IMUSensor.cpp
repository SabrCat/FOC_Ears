// IMUSensor.cpp

#include "IMUSensor.h"

// ── begin() ──────────────────────────────────────────────────────────────────

bool IMUSensor::begin(TwoWire  &wire,
                      uint8_t   addr,
                      int16_t calXA, int16_t calYA, int16_t calZA,
                      int16_t calXG, int16_t calYG, int16_t calZG,
                      float sampleHz,
                      float yawDecay)
{
    _wire     = &wire;
    _addr     = addr;
    _yawDecay = yawDecay;

    // Adafruit begin() issues a full hardware reset, zeroing offset registers.
    // Everything that writes to the chip must come AFTER this call.
    if (!_mpu.begin(addr, &wire)) return false;

    // Ranges must exactly match what was set during calibration.
    // Calibration sketch uses: ±2 g, ±250°/s.
    // Changing either here invalidates stored offsets.
    _mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
    _mpu.setGyroRange(MPU6050_RANGE_250_DEG);

    // Re-apply calibration offsets.  The chip subtracts these from every ADC
    // result before outputting, so readMotion6() returns bias-free data
    // automatically — no software subtraction step needed.
    writeReg16(REG_XA_OFFS_H,    calXA);
    writeReg16(REG_YA_OFFS_H,    calYA);
    writeReg16(REG_ZA_OFFS_H,    calZA);
    writeReg16(REG_XG_OFFS_USRH, calXG);
    writeReg16(REG_YG_OFFS_USRH, calYG);
    writeReg16(REG_ZG_OFFS_USRH, calZG);

    // Tell the filter how fast it will be called so it can set dt = 1/sampleHz.
    _filter.begin(sampleHz);

    return true;
}

// ── update() ─────────────────────────────────────────────────────────────────

bool IMUSensor::update()
{
    int16_t axr, ayr, azr, gxr, gyr, gzr;
    const bool ok = readMotion6(axr, ayr, azr, gxr, gyr, gzr);
    // Even on a glitch the filter advances on gyro data (Mahony's own guard
    // skips the accel correction when accel is all-zero), so don't return early.

    // Raw sensor counts → physical units
    const float ax_s = axr / ACCEL_SCALE;   // sensor frame, [g]
    const float ay_s = ayr / ACCEL_SCALE;
    const float az_s = azr / ACCEL_SCALE;
    const float gx_s = gxr / GYRO_SCALE;    // sensor frame, [°/s]
    const float gy_s = gyr / GYRO_SCALE;    // NOTE: updateIMU() converts °/s → rad/s internally —
    const float gz_s = gzr / GYRO_SCALE;    //       pass °/s, NOT rad/s.

    // ── Axis remap ────────────────────────────────────────────────────────────
    // The MPU6050 is mounted with a +90° rotation around Z relative to
    // Mahony's expected frame.  Sensor-Y points physically forward (roll axis)
    // and sensor-X points physically sideways (pitch axis).  Without this
    // correction roll and pitch are swapped in the output.
    //
    // Physical frame = R(+90° Z) · sensor frame:
    //
    //   phys X (forward / roll axis)  = −sensor Y
    //   phys Y (lateral / pitch axis) =  sensor X
    //   phys Z (up / yaw axis)        =  sensor Z   ← yaw unaffected
    //
    // To undo this if you remount the board so axes align naturally, remove
    // this block and replace ax/ay/gx/gy with ax_s/ay_s/gx_s/gy_s below.
    const float ax = -ay_s;
    const float ay =  ax_s;
    const float az =  az_s;
    const float gx = -gy_s;
    const float gy =  gx_s;
          float gz =  gz_s;

    // ── Yaw zero-bias ─────────────────────────────────────────────────────────
    // Inject a corrective yaw rate proportional to the current heading error.
    // Evaluated on the PREVIOUS frame's quaternion — no algebraic loop.
    //
    // Corrective rate [°/s] = −yawDecay [1/s] × yaw [rad] × (180/π)
    //   → decays like yaw(t) = yaw₀ · exp(−yawDecay · t)
    //   → time constant = 1/yawDecay  (0.05 → 20 s,  0.2 → 5 s)
    if (_yawDecay > 0.0f) {
        const float yawErrRad = _filter.getYawRadians();           // [-π, +π]
        gz -= yawErrRad * _yawDecay * (180.0f / M_PI);
    }

    _filter.updateIMU(gx, gy, gz, ax, ay, az);

    // ── Populate output state (physical frame throughout) ─────────────────────
    _s.ax = ax;
    _s.ay = ay;
    _s.az = az;

    // Physical-frame gyro rates, before yaw correction, so the caller sees
    // what the sensor measured expressed in the physical frame.
    _s.gx = -gy_s;
    _s.gy =  gx_s;
    _s.gz =  gz_s;

    // getRoll() / getPitch() return degrees directly.
    _s.roll  = _filter.getRoll();
    _s.pitch = _filter.getPitch();

    // getYaw() returns (yaw_rad × 57.29578 + 180°), i.e. [0°, 360°].
    // Use getYawRadians() instead for a zero-centred [-180°, +180°] value.
    _s.yaw = _filter.getYawRadians() * (180.0f / M_PI);

    return ok;
}

// ── Private helpers ───────────────────────────────────────────────────────────

void IMUSensor::writeReg16(uint8_t reg, int16_t val)
{
    _wire->beginTransmission(_addr);
    _wire->write(reg);
    _wire->write(static_cast<uint8_t>(val >> 8));
    _wire->write(static_cast<uint8_t>(val & 0xFF));
    _wire->endTransmission();
}

bool IMUSensor::readMotion6(int16_t &ax, int16_t &ay, int16_t &az,
                             int16_t &gx, int16_t &gy, int16_t &gz)
{
    _wire->beginTransmission(_addr);
    _wire->write(REG_ACCEL_XOUT_H);
    if (_wire->endTransmission(false) != 0) return false;
    if (_wire->requestFrom(_addr, static_cast<uint8_t>(14)) != 14) return false;

    // Safe big-endian 16-bit read: shift unsigned to avoid implementation-defined
    // behaviour on negative high bytes, then reinterpret as signed.
    auto readWord = [this]() -> int16_t {
        const uint8_t hi = _wire->read(), lo = _wire->read();
        return static_cast<int16_t>(static_cast<uint16_t>(hi) << 8 | lo);
    };

    ax = readWord();  ay = readWord();  az = readWord();
    readWord();                          // temperature — discard
    gx = readWord();  gy = readWord();  gz = readWord();

    // All-zero accelerometer is the Mahony filter's own guard condition for
    // skipping the gravity correction.  Signal the caller so they can decide
    // whether to discard the frame.
    return !((ax == 0) && (ay == 0) && (az == 0));
}