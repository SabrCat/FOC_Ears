/*
  MPU6050 Calibration — Adafruit library port
  Original: ElectronicCats/MPU6050 IMU_Zero example
  https://registry.platformio.org/libraries/electroniccats/MPU6050

  Changes from original:
    - Replaced ElectronicCats MPU6050 class with Adafruit_MPU6050
    - getMotion6() / setX/YAccelOffset() / setX/YGyroOffset() reimplemented
      via direct I2C register access (the Adafruit library does not expose them)
    - CalibrateAccel() / CalibrateGyro() PID pre-pass omitted — the
      binary-search below (PullBracketsOut → PullBracketsIn) is self-contained
      and produces equivalent final offsets without it
    - Accelerometer range forced to ±2 g so that 1 g = 16384 LSB, matching
      Target[iAz] = 16384 (the Adafruit library defaults to ±8 g)

  platformio.ini lib_deps:
    adafruit/Adafruit MPU6050
    adafruit/Adafruit Unified Sensor
*/

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

#define I2C_SDA      17
#define I2C_SCL      18
#define MPU_ADDR     0x68   // change to 0x69 if AD0 is pulled high

Adafruit_MPU6050 mpu;

// ── MPU6050 register map (raw I2C access) ─────────────────────────
// Raw sensor output: 14 consecutive bytes starting at 0x3B
//   [AX_H AX_L  AY_H AY_L  AZ_H AZ_L  TEMP_H TEMP_L  GX_H GX_L  GY_H GY_L  GZ_H GZ_L]
#define REG_ACCEL_XOUT_H  0x3B

// User-programmable offset registers (each is a 16-bit signed value, big-endian)
#define REG_XA_OFFS_H     0x06
#define REG_YA_OFFS_H     0x08
#define REG_ZA_OFFS_H     0x0A
#define REG_XG_OFFS_USRH  0x13
#define REG_YG_OFFS_USRH  0x15
#define REG_ZG_OFFS_USRH  0x17

// ── Calibration constants ─────────────────────────────────────────
const int usDelay             = 3150;   // µs between samples → ~200 Hz
const int NFast               = 1000;
const int NSlow               = 10000;
const int LinesBetweenHeaders = 5;

const int iAx = 0, iAy = 1, iAz = 2;
const int iGx = 3, iGy = 4, iGz = 5;

int LowValue[6], HighValue[6], Smoothed[6];
int LowOffset[6], HighOffset[6], Target[6];
int LinesOut, N, i;

// ── Raw I2C helpers ───────────────────────────────────────────────

static void writeReg16(uint8_t reg, int16_t value) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write((uint8_t)(value >> 8));
  Wire.write((uint8_t)(value & 0xFF));
  Wire.endTransmission();
}

// ── Drop-in replacements for ElectronicCats methods ──────────────

// Equivalent to mpu.getMotion6(ax, ay, az, gx, gy, gz)
void getMotion6(int16_t *ax, int16_t *ay, int16_t *az,
                int16_t *gx, int16_t *gy, int16_t *gz) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(REG_ACCEL_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)14);
  *ax = ((int16_t)Wire.read() << 8) | Wire.read();
  *ay = ((int16_t)Wire.read() << 8) | Wire.read();
  *az = ((int16_t)Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read();                           // temperature — discard
  *gx = ((int16_t)Wire.read() << 8) | Wire.read();
  *gy = ((int16_t)Wire.read() << 8) | Wire.read();
  *gz = ((int16_t)Wire.read() << 8) | Wire.read();
}

// Equivalent to mpu.setX/YAccelOffset() + mpu.setX/YGyroOffset()
void setAllOffsets(int offsets[6]) {
  writeReg16(REG_XA_OFFS_H,    (int16_t)offsets[iAx]);
  writeReg16(REG_YA_OFFS_H,    (int16_t)offsets[iAy]);
  writeReg16(REG_ZA_OFFS_H,    (int16_t)offsets[iAz]);
  writeReg16(REG_XG_OFFS_USRH, (int16_t)offsets[iGx]);
  writeReg16(REG_YG_OFFS_USRH, (int16_t)offsets[iGy]);
  writeReg16(REG_ZG_OFFS_USRH, (int16_t)offsets[iGz]);
}

// Equivalent to mpu.PrintActiveOffsets()
void printActiveOffsets() {
  const uint8_t regs[6] = {
    REG_XA_OFFS_H, REG_YA_OFFS_H, REG_ZA_OFFS_H,
    REG_XG_OFFS_USRH, REG_YG_OFFS_USRH, REG_ZG_OFFS_USRH
  };
  Serial.print("Active offsets [XA YA ZA XG YG ZG]: ");
  for (int k = 0; k < 6; k++) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(regs[k]);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)2);
    int16_t v = ((int16_t)Wire.read() << 8) | Wire.read();
    Serial.print(v);
    if (k < 5) Serial.print("\t");
  }
  Serial.println();
}

// ── Algorithm (unchanged from original) ──────────────────────────

void ForceHeader() { LinesOut = 99; }

void GetSmoothed() {
  int16_t RawValue[6];
  long    Sums[6];
  for (i = 0; i <= 5; i++) Sums[i] = 0;
  for (i = 1; i <= N; i++) {
    getMotion6(&RawValue[iAx], &RawValue[iAy], &RawValue[iAz],
               &RawValue[iGx], &RawValue[iGy], &RawValue[iGz]);
    delayMicroseconds(usDelay);
    for (int j = 0; j <= 5; j++) Sums[j] += RawValue[j];
  }
  for (i = 0; i <= 5; i++) Smoothed[i] = (Sums[i] + N / 2) / N;
}

void SetOffsets(int TheOffsets[6]) { setAllOffsets(TheOffsets); }

void ShowProgress() {
  if (LinesOut >= LinesBetweenHeaders) {
    Serial.println("\t\tXAccel\t\t\tYAccel\t\t\t\tZAccel\t\t\tXGyro\t\t\tYGyro\t\t\tZGyro");
    LinesOut = 0;
  }
  Serial.print(' ');
  for (i = 0; i <= 5; i++) {
    Serial.print('[');
    Serial.print(LowOffset[i]);
    Serial.print(',');
    Serial.print(HighOffset[i]);
    Serial.print("] --> [");
    Serial.print(LowValue[i]);
    Serial.print(',');
    Serial.print(HighValue[i]);
    if (i == 5) Serial.println("]");
    else        Serial.print("]\t");
  }
  LinesOut++;
}

void SetAveraging(int NewN) {
  N = NewN;
  Serial.print("\nAveraging ");
  Serial.print(N);
  Serial.println(" readings each time");
}

void PullBracketsOut() {
  boolean Done = false;
  int NextLowOffset[6], NextHighOffset[6];

  Serial.println("Expanding:");
  ForceHeader();

  while (!Done) {
    Done = true;

    SetOffsets(LowOffset);
    GetSmoothed();
    for (i = 0; i <= 5; i++) {
      LowValue[i] = Smoothed[i];
      if (LowValue[i] >= Target[i]) { Done = false; NextLowOffset[i] = LowOffset[i] - 1000; }
      else                           NextLowOffset[i] = LowOffset[i];
    }

    SetOffsets(HighOffset);
    GetSmoothed();
    for (i = 0; i <= 5; i++) {
      HighValue[i] = Smoothed[i];
      if (HighValue[i] <= Target[i]) { Done = false; NextHighOffset[i] = HighOffset[i] + 1000; }
      else                            NextHighOffset[i] = HighOffset[i];
    }

    ShowProgress();
    for (int i = 0; i <= 5; i++) {
      LowOffset[i]  = NextLowOffset[i];
      HighOffset[i] = NextHighOffset[i];
    }
  }
}

void PullBracketsIn() {
  boolean AllBracketsNarrow, StillWorking;
  int NewOffset[6];

  Serial.println("\nClosing in:");
  AllBracketsNarrow = false;
  ForceHeader();
  StillWorking = true;

  while (StillWorking) {
    StillWorking = false;
    if (AllBracketsNarrow && (N == NFast)) SetAveraging(NSlow);
    else                                   AllBracketsNarrow = true;

    for (int i = 0; i <= 5; i++) {
      if (HighOffset[i] <= (LowOffset[i] + 1)) {
        NewOffset[i] = LowOffset[i];
      } else {
        StillWorking = true;
        NewOffset[i] = (LowOffset[i] + HighOffset[i]) / 2;
        if (HighOffset[i] > (LowOffset[i] + 10)) AllBracketsNarrow = false;
      }
    }

    SetOffsets(NewOffset);
    GetSmoothed();
    for (i = 0; i <= 5; i++) {
      if (Smoothed[i] > Target[i]) { HighOffset[i] = NewOffset[i]; HighValue[i] = Smoothed[i]; }
      else                         { LowOffset[i]  = NewOffset[i]; LowValue[i]  = Smoothed[i]; }
    }
    ShowProgress();
  }
}

// ── Init ──────────────────────────────────────────────────────────

void Initialize() {
  Wire.begin(I2C_SDA, I2C_SCL);
  Serial.begin(9600);

  Serial.println("Initializing MPU6050...");
  if (!mpu.begin(MPU_ADDR, &Wire)) {
    Serial.println("MPU6050 connection failed — check wiring and address");
    while (true) { delay(10); }
  }
  Serial.println("MPU6050 connection successful");

  // ±2 g  →  16384 LSB/g  (required so Target[iAz] == 16384 == 1 g)
  // ±8 g is the Adafruit library default, which would break the target value
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);

  Serial.println("\nNote: PID pre-calibration step from original omitted.");
  Serial.println("The binary search below produces equivalent final offsets.\n");
}

// ── Arduino entry points ──────────────────────────────────────────

void setup() {
  Initialize();

  for (i = iAx; i <= iGz; i++) {
    Target[i]     = 0;
    HighOffset[i] = 0;
    LowOffset[i]  = 0;
  }
  Target[iAz] = 16384;  // 1 g at ±2 g full-scale (16384 LSB/g)

  SetAveraging(NFast);
  PullBracketsOut();
  PullBracketsIn();

  Serial.println("-------------- DONE --------------");
  printActiveOffsets();
}

void loop() {
  // nothing — paste the reported offsets into your main sketch
}