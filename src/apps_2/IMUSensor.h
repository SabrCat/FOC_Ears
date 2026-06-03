#pragma once
// IMUSensor.h
// Wraps MPU6050 (via Adafruit library for init) + paulstoffregen/Mahony v1.2
// into a single class that exposes calibrated accelerations, angular rates,
// and Mahony-filtered roll/pitch/yaw.
//
// lib_deps required in platformio.ini:
//   adafruit/Adafruit MPU6050
//   adafruit/Adafruit Unified Sensor
//   paulstoffregen/Mahony @ ^1.2.0

#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <MahonyAHRS.h>

class IMUSensor {
public:

    // All outputs in one struct for easy passing around.
    struct State {
        float ax, ay, az;    // Linear acceleration  [g]
        float gx, gy, gz;    // Angular rate         [°/s]  (raw, pre-correction)
        float roll;          // [°]  gravity-stabilised, never drifts
        float pitch;         // [°]  gravity-stabilised, never drifts
        float yaw;           // [°]  zero-biased (see yawDecay), range [-180, +180]
    };

    // begin() — call once in setup(), after Wire.begin().
    //
    //   wire        — I²C bus (Wire, Wire1, …)
    //   addr        — 0x68 (default) or 0x69 (AD0 high)
    //   calXA..ZG   — six int16_t values from the calibration sketch output line
    //                 "Active offsets [XA YA ZA XG YG ZG]:"
    //   sampleHz    — how fast you will call update() — must match exactly.
    //                 Sets Mahony's integration step to 1/sampleHz seconds.
    //   yawDecay    — restoring-force gain toward yaw = 0  [1/s]
    //                 Time constant ≈ 1/yawDecay seconds.
    //                 0.0  → pure gyro integration (unlimited drift)
    //                 0.05 → ~20 s  (default — allows short-term turns,
    //                                prevents overflow, zero is resting position)
    //                 0.2  → ~5 s   (snaps back quickly)
    //
    // Returns false if the MPU6050 does not respond.
    bool begin(TwoWire  &wire,
               uint8_t   addr,
               int16_t calXA, int16_t calYA, int16_t calZA,
               int16_t calXG, int16_t calYG, int16_t calZG,
               float sampleHz = 200.0f,
               float yawDecay = 0.05f);

    // update() — call at exactly sampleHz inside a fixed-interval loop.
    //            Do NOT use delay(); use the micros() + lastUpdate += pattern.
    //
    // Returns false if the I²C read failed (all-zero accelerometer guard).
    // The Mahony filter is still advanced on gyro data when this happens.
    bool update();

    // Individual accessors (also available via state())
    const State& state()  const { return _s; }
    float getRoll()       const { return _s.roll;  }
    float getPitch()      const { return _s.pitch; }
    float getYaw()        const { return _s.yaw;   }
    float getAccelX()     const { return _s.ax;    }
    float getAccelY()     const { return _s.ay;    }
    float getAccelZ()     const { return _s.az;    }
    float getGyroX()      const { return _s.gx;    }
    float getGyroY()      const { return _s.gy;    }
    float getGyroZ()      const { return _s.gz;    }

private:
    // Scale factors — tied to the ranges set in begin() and used during calibration
    static constexpr float ACCEL_SCALE = 16384.0f; // ±2 g   →  16384 LSB/g
    static constexpr float GYRO_SCALE  =   131.0f; // ±250°/s →   131 LSB/(°/s)

    // MPU6050 register map
    static constexpr uint8_t REG_ACCEL_XOUT_H = 0x3B;
    static constexpr uint8_t REG_XA_OFFS_H    = 0x06;
    static constexpr uint8_t REG_YA_OFFS_H    = 0x08;
    static constexpr uint8_t REG_ZA_OFFS_H    = 0x0A;
    static constexpr uint8_t REG_XG_OFFS_USRH = 0x13;
    static constexpr uint8_t REG_YG_OFFS_USRH = 0x15;
    static constexpr uint8_t REG_ZG_OFFS_USRH = 0x17;

    Adafruit_MPU6050 _mpu;
    Mahony           _filter;
    TwoWire         *_wire    = nullptr;
    uint8_t          _addr    = 0x68;
    float            _yawDecay = 0.05f;
    State            _s{};

    void writeReg16(uint8_t reg, int16_t val);
    bool readMotion6(int16_t &ax, int16_t &ay, int16_t &az,
                     int16_t &gx, int16_t &gy, int16_t &gz);
};
