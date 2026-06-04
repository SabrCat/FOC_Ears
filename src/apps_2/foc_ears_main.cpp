#include "Arduino.h"
#include "esp_task_wdt.h"
#include "esp_system.h"
#include "SPI.h"
#include "SimpleFOC.h"
#include "SimpleFOCDrivers.h"
#include "encoders/MT6701/MagneticSensorMT6701SSI.h"
#include <FastLED.h>
#include <VL53L1X.h>
#include "IMUSensor.h"
#include "EarMotor.h"

// ============================================================================
#pragma region CONFIGURATION
// ============================================================================

// Pins - I2C
#define I2C_SDA 17
#define I2C_SCL 18

// Pins - SPI (Angle Sensors)
#define SENSOR_SPI_CLK 9
#define SENSOR_SPI_MISO 10
#define SENSOR_SPI_MOSI 11 // MOSI is not used but must be specified

// Pins - Board 1
#define B1_SENSOR_CS_PIN 12
#define B1_MOTOR_PWM_A 13
#define B1_MOTOR_PWM_B 14
#define B1_MOTOR_PWM_C 15
#define B1_MOTOR_ENABLE 16

// Pins - Board 2
#define B2_SENSOR_CS_PIN 48
#define B2_MOTOR_PWM_A 26
#define B2_MOTOR_PWM_B 47
#define B2_MOTOR_PWM_C 33
#define B2_MOTOR_ENABLE 34

// Pins - I/O
#define BOOT_BUTTON_PIN 0
#define RGB_LED_PIN 8
#define MPU_ADDR 0x68

// Motor Electrical
#define MOTOR_POLE_PAIRS 7           // Mitoot 2804: 7 pole pairs
#define MOTOR_PHASE_RESISTANCE 3.25f // Ohms (6.5 / 2)
#define MOTOR_KV 330                 // RPM/V

// Driver / Controller
#define VOLTAGE_POWER_SUPPLY 12.0f
#define PWM_FREQUENCY 40000
#define TARGET_FOC_LOOP_HZ 10000 // target FOC loop rate in Hz; 0 = unlimited

// Command LPF cutoff — smooths position steps from the 200 Hz animation task
// at the 10 kHz FOC rate. Tf = 1/(2π·fc) is computed in setup() from this value.
// Lower = quieter/smoother (more Tf), higher = more responsive (less Tf).
// Default 0.02 s ≈ 8 Hz. Go lower to reduce noise: 5 Hz → 0.032 s, 3 Hz → 0.053 s.
static constexpr float COMMAND_LPF_HZ = 8.0f;

// ============================================================================
#pragma endregion

// ============================================================================
#pragma region HARDWARE
// ============================================================================

// IMU Calibration offsets
// From calibration sketch final output: "Active offsets [XA YA ZA XG YG ZG]:"
static constexpr int16_t CAL_XA = -3181;
static constexpr int16_t CAL_YA = -761;
static constexpr int16_t CAL_ZA = 455;
static constexpr int16_t CAL_XG = 54;
static constexpr int16_t CAL_YG = -7;
static constexpr int16_t CAL_ZG = 4;

// IMU Sample rate (must match value passed to begin())
static constexpr float SAMPLE_HZ = 200.0f;

// Sensor Objects
IMUSensor imu;
VL53L1X distanceSensor;

// Motor Objects — mot1 = right ear (Board 1), mot2 = left ear (Board 2)
static EarMotor *mot1 = nullptr;
static EarMotor *mot2 = nullptr;

// LED
CRGB onboard_led[1];

// System status — written from setup() or any task; uint8_t writes are atomic on ESP32.
enum class SystemStatus : uint8_t
{
    STARTING,     // dim white
    INIT_TOF,     // cyan
    INIT_IMU,     // blue
    INIT_MOTOR_1, // yellow — right ear aligning
    INIT_MOTOR_2, // orange — left ear aligning
    RUNNING,      // green breathing (thermal auto-overrides in LED_Task)
    ERROR,        // solid red — stuck in while(true), LED_Task never starts
};
volatile SystemStatus g_status = SystemStatus::STARTING;

// ============================================================================
#pragma endregion

// ============================================================================
#pragma region TASKS
// ============================================================================

// Thermal thresholds for LED warning override during RUNNING state.
static constexpr float LED_THERMAL_CAUTION_RATIO = 0.5f;  // tripRatio above this → amber
static constexpr float LED_THERMAL_CRITICAL_VOLTS = 5.0f; // voltageLimit below this → red pulse

#pragma region LED_Task

// Derives LED color from g_status and motor thermal state, then calls FastLED.show().
// Called from LED_Task at 10 Hz, and directly from setup() after each g_status change
// so init stages are immediately visible without starting the task early.
void updateStatusLED()
{
    const float t = (float)millis() * 1e-3f;

    // Thermal override — only when motors are initialised and system is running.
    if (g_status == SystemStatus::RUNNING && mot1 != nullptr && mot2 != nullptr)
    {
        const float minVoltLimit = min(mot1->diagState().voltageLimit,
                                       mot2->diagState().voltageLimit);
        const float maxTripRatio = max(mot1->diagState().tripRatio,
                                       mot2->diagState().tripRatio);

        if (minVoltLimit < LED_THERMAL_CRITICAL_VOLTS)
        {
            float pulse = 0.5f + 0.5f * sinf(2.0f * PI * 2.0f * t);
            onboard_led[0] = CRGB((uint8_t)(pulse * 200.0f), 0, 0);
            FastLED.show();
            return;
        }
        if (maxTripRatio > LED_THERMAL_CAUTION_RATIO)
        {
            onboard_led[0] = CRGB(200, 60, 0);
            FastLED.show();
            return;
        }
    }

    switch (g_status)
    {
    case SystemStatus::STARTING:
        onboard_led[0] = CRGB(40, 40, 40);
        break;
    case SystemStatus::INIT_TOF:
        onboard_led[0] = CRGB(0, 200, 200);
        break;
    case SystemStatus::INIT_IMU:
        onboard_led[0] = CRGB(0, 0, 200);
        break;
    case SystemStatus::INIT_MOTOR_1:
        onboard_led[0] = CRGB(200, 200, 0);
        break;
    case SystemStatus::INIT_MOTOR_2:
        onboard_led[0] = CRGB(255, 100, 0);
        break;
    case SystemStatus::RUNNING:
    {
        float breath = 0.9f + 0.15f * sinf(2.0f * PI / 2.0f * t);
        onboard_led[0] = CRGB(0, (uint8_t)(breath * 20.0f), 0);
        break;
    }
    case SystemStatus::ERROR:
        onboard_led[0] = CRGB(200, 0, 0);
        break;
    }
    FastLED.show();
}

// LED Task — drives the status LED at 10 Hz during normal operation.
void LED_Task(void *param)
{
    for (;;)
    {
        vTaskDelay(pdMS_TO_TICKS(100));
        updateStatusLED();
    }
}
#pragma endregion

// Animation Task — IMU + ToF + animation driver, runs at SAMPLE_HZ on core 1.
#pragma region Anim_Task
void Anim_Task(void *param)
{
    // ── Tuning — all animation knobs live here, next to the code that reads them ──

    const float MOTOR_THROW = 2.5f; // rad — hardware limit, matches EarMotorConfig

    // IMU → ear position gains
    const float ROLL_GAIN = 0.018f;   // rad ear offset per degree of head roll
    const float PITCH_GAIN = -0.010f; // rad ear offset per degree of head pitch (flip sign if wrong)

    // Gait index: bandpass lateral accel (HPF 1 Hz / LPF 4 Hz) → asymmetric EMA
    const float GAIT_HPF_HZ = 1.0f;      // Hz  — kills slow tilt drift
    const float GAIT_LPF_HZ = 4.0f;      // Hz  — smooths step-to-step spikes
    const float GAIT_ATTACK_TAU = 1.0f;  // s   — rise time (slow = needs sustained walking)
    const float GAIT_DECAY_TAU = 1.5f;   // s   — fall time between steps
    const float GAIT_SATURATION = 0.04f; // g   — min(envLay,envLaz) at "fully walking" (tune this)

    // Nod index: pitch gyro magnitude (gy) → asymmetric EMA
    const float NOD_ATTACK_TAU = 0.10f;  // s   — snappy social response
    const float NOD_DECAY_TAU = 1.0f;    // s   — lingers after activity stops
    const float NOD_SATURATION = 150.0f; // °/s — gy level at "fully nodding" (tune this)

    // Look index: yaw gyro magnitude (gz) → asymmetric EMA (head shake / scan)
    const float LOOK_ATTACK_TAU = 0.15f;  // s
    const float LOOK_DECAY_TAU = 0.75f;   // s
    const float LOOK_SATURATION = 800.0f; // °/s — gz level at "actively scanning" (tune this)

    // Social layer: (nod + look), suppressed by gait → ears perk forward
    const float SOCIAL_DEPTH = 1.5f;        // rad shift toward forward at full social
    const float GAIT_SUPPRESS_START = 0.3f; // gait below this has zero effect on social

    // Movement bob layer: gait → periodic ear oscillation
    const float MOVEMENT_DEPTH = 0.25f; // rad amplitude
    const float MOVEMENT_FREQ = 1.8f;   // Hz  (walking cadence)

    // (flick + headpat constants are local to their blocks below)

    // ── Filter & index state ──────────────────────────────────────────────────
    float hpLay = 0.0f, prevLay = 0.0f;
    float hpLaz = 0.0f, prevLaz = 0.0f;
    float bpLay = 0.0f, bpLaz = 0.0f;
    float gaitEnvLay = 0.0f, gaitEnvLaz = 0.0f; // separate envelopes, min'd into gaitIdx
    float gaitIdx = 0.0f;
    float nodIdx = 0.0f;
    float lookIdx = 0.0f;

    float distSmoothed = 150.0f; // starts at threshold — see headpat block
    float headpatIntensity = 0.0f;
    uint32_t rightFlickStart = 0;
    uint32_t leftFlickStart = 0;
    float rightFlickAmplitude = 1.0f;
    float leftFlickAmplitude = 1.0f;

    float rightListenIntensity = 0.0f;
    float rightListenTarget = 0.0f; // rad in [0.2, 2.3] — rightward targets only
    float rightListenRefYaw = 0.0f;
    uint32_t rightListenHoldUntil = 0;
    bool rightListenActive = false;

    float leftListenIntensity = 0.0f;
    float leftListenTarget = 0.0f; // rad in [-2.3, -0.2] — leftward targets only
    float leftListenRefYaw = 0.0f;
    uint32_t leftListenHoldUntil = 0;
    bool leftListenActive = false;

    // ── Derived constants (computed once from tuning values above) ────────────
    const float dt = 1.0f / SAMPLE_HZ;
    const float hpAlpha = 1.0f / (1.0f + dt * 2.0f * PI * GAIT_HPF_HZ);
    const float bpLpAlpha = dt * 2.0f * PI * GAIT_LPF_HZ;
    const float gaitAttackAlpha = dt / GAIT_ATTACK_TAU;
    const float gaitDecayAlpha = dt / GAIT_DECAY_TAU;
    const float nodAttackAlpha = dt / NOD_ATTACK_TAU;
    const float nodDecayAlpha = dt / NOD_DECAY_TAU;
    const float lookAttackAlpha = dt / LOOK_ATTACK_TAU;
    const float lookDecayAlpha = dt / LOOK_DECAY_TAU;
    TickType_t xLastWakeTime = xTaskGetTickCount();

    for (;;)
    {
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(5)); // 200 Hz

        // ── Sensor reads ─────────────────────────────────────────────────────

        imu.update();
        const IMUSensor::State &imuState = imu.state();

        // Serial.printf(">ax:%f\n", imuState.ax);
        // Serial.printf(">ay:%f\n", imuState.ay);
        // Serial.printf(">az:%f\n", imuState.az);

        // Serial.printf(">gx:%f\n", imuState.gx);
        // Serial.printf(">gy:%f\n", imuState.gy);
        // Serial.printf(">gz:%f\n", imuState.gz);

        // Serial.printf(">pitch:%f\n", imuState.pitch);
        // Serial.printf(">roll:%f\n", imuState.roll);
        // Serial.printf(">yaw:%f\n", imuState.yaw);

        // ToF: only consume a new measurement when one is ready.
        // EMA smoothing filters jitter while close; clamp keeps distSmoothed at the
        // threshold when nothing is nearby so the filter responds instantly on approach.
        if (distanceSensor.dataReady())
        {
            const float HEADPAT_THRESHOLD_MM = 150.0f;
            const float HEADPAT_ATTACK_ALPHA = 0.25f; // τ ≈ 20 ms at 200 Hz
            float rawDist = (float)distanceSensor.read(false);
            distSmoothed += (rawDist - distSmoothed) * HEADPAT_ATTACK_ALPHA;
            if (distSmoothed > HEADPAT_THRESHOLD_MM)
                distSmoothed = HEADPAT_THRESHOLD_MM;
        }

        // ── Activity indices ──────────────────────────────────────────────────

        // Gait: require BOTH lateral and vertical envelopes to be elevated.
        // Head turns spike lay but not laz. Sit/stand spikes laz but not lay.
        // min() of the two envelopes rejects both false-positive sources.
        hpLay = hpAlpha * (hpLay + imuState.lay - prevLay);
        prevLay = imuState.lay;
        hpLaz = hpAlpha * (hpLaz + imuState.laz - prevLaz);
        prevLaz = imuState.laz;
        bpLay += (hpLay - bpLay) * bpLpAlpha;
        bpLaz += (hpLaz - bpLaz) * bpLpAlpha;
        float laySig = fabsf(bpLay), lazSig = fabsf(bpLaz);
        gaitEnvLay += (laySig - gaitEnvLay) * ((laySig > gaitEnvLay) ? gaitAttackAlpha : gaitDecayAlpha);
        gaitEnvLaz += (lazSig - gaitEnvLaz) * ((lazSig > gaitEnvLaz) ? gaitAttackAlpha : gaitDecayAlpha);
        gaitIdx = min(gaitEnvLay, gaitEnvLaz);

        // Nod: pitch gyro magnitude → asymmetric EMA
        float nodSig = fabsf(imuState.gy);
        nodIdx += (nodSig - nodIdx) * ((nodSig > nodIdx) ? nodAttackAlpha : nodDecayAlpha);

        // Look: yaw gyro magnitude → asymmetric EMA
        float lookSig = fabsf(imuState.gz);
        lookIdx += (lookSig - lookIdx) * ((lookSig > lookIdx) ? lookAttackAlpha : lookDecayAlpha);

        float gaitNorm = constrain(gaitIdx / GAIT_SATURATION, 0.0f, 1.0f);
        float nodNorm = sqrtf(constrain(nodIdx / NOD_SATURATION, 0.0f, 1.0f));
        float lookNorm = sqrtf(constrain(lookIdx / LOOK_SATURATION, 0.0f, 1.0f));
        float gaitSuppress = constrain((gaitNorm - GAIT_SUPPRESS_START) / (1.0f - GAIT_SUPPRESS_START), 0.0f, 1.0f);
        float socialNorm = constrain(nodNorm + lookNorm, 0.0f, 1.0f) * (1.0f - gaitSuppress);

        // Serial.printf(">gaitLay:%f\n", gaitEnvLay / GAIT_SATURATION);
        // Serial.printf(">gaitLaz:%f\n", gaitEnvLaz / GAIT_SATURATION);
        // Serial.printf(">gait:%f\n", gaitNorm);
        // Serial.printf(">nod:%f\n", nodNorm);
        // Serial.printf(">look:%f\n", lookNorm);
        // Serial.printf(">social:%f\n", socialNorm);

        // ── Animation layers ──────────────────────────────────────────────────

        const float base = 1.5;

        float rightTarget = base;
        float leftTarget = base;

        // Social — ears perk forward with conversation activity, suppressed while walking.
        float socialOffset = -socialNorm * 1.0f;
        rightTarget += socialOffset;
        leftTarget += socialOffset;

        float nodMotionOffset = imuState.gy / 200.0f * 0.2f * socialNorm;
        rightTarget += nodMotionOffset;
        leftTarget += nodMotionOffset;

        float headRotationLookaheadOffset = imuState.gz / 100.0f * 0.2f;
        rightTarget += -constrain(headRotationLookaheadOffset, -1.0f, 0);
        leftTarget += constrain(headRotationLookaheadOffset, 0, 1.0f);

        float walkingAccelerationOffset = imuState.laz * 2.0f * 0.5f * gaitSuppress;
        walkingAccelerationOffset = constrain(walkingAccelerationOffset, -0.3f, 0.3f);
        rightTarget += walkingAccelerationOffset;
        leftTarget += walkingAccelerationOffset;

        float rollTiltOffset = imuState.roll / 45.0f;
        rightTarget += constrain(rollTiltOffset, 0, 1);
        leftTarget += -constrain(rollTiltOffset, -1, 0);

        float pitchTiltOffset = constrain(-(imuState.pitch + 20) / 45, 0, 1);
        rightTarget += pitchTiltOffset;
        leftTarget += pitchTiltOffset;
        // Serial.printf(">PitchOffset:%f\n", pitchTiltOffset);

        // Headpat — asymmetric EMA on proximity: fast attack, slow release.
        // distSmoothed is clamped to threshold when the hand is absent, so rawProximity
        // snaps to 0 instantly on dropout; headpatIntensity then decays via RELEASE_ALPHA.
        {
            const float HEADPAT_THRESHOLD_MM = 150.0f;
            const float HEADPAT_DEPTH = 1.0f;             // rad toward forward at full contact
            const float HEADPAT_ATTACK_ALPHA = 0.5f;      // τ ≈ 20 ms at 200 Hz
            const float HEADPAT_RELEASE_ALPHA = 0.02f;    // τ ≈ 250 ms at 200 Hz
            const float HEADPAT_FLUTTER_THRESHOLD = 0.7f; // intensity fraction where flutter starts
            const float HEADPAT_FLUTTER_FREQ_HZ = 10.0f;
            const float HEADPAT_FLUTTER_AMPLITUDE = 0.3f; // rad
            const float HEADPAT_FLUTTER_PHASE = 0.8f;     // rad offset between ears

            float rawProximity = 1.0f - distSmoothed / HEADPAT_THRESHOLD_MM;
            float headpatAlpha = (rawProximity > headpatIntensity) ? HEADPAT_ATTACK_ALPHA : HEADPAT_RELEASE_ALPHA;
            headpatIntensity += (rawProximity - headpatIntensity) * headpatAlpha;

            float headpatOffset = -headpatIntensity * HEADPAT_DEPTH;

            // Flutter — oscillation envelope opens only at high intensity (very close contact).
            float flutterEnv = constrain(
                (headpatIntensity - HEADPAT_FLUTTER_THRESHOLD) / (1.0f - HEADPAT_FLUTTER_THRESHOLD),
                0.0f, 1.0f);
            float t = (float)millis() * 1e-3f;
            float flutterRight = flutterEnv * HEADPAT_FLUTTER_AMPLITUDE *
                                 sinf(2.0f * PI * HEADPAT_FLUTTER_FREQ_HZ * t);
            float flutterLeft = flutterEnv * HEADPAT_FLUTTER_AMPLITUDE *
                                sinf(2.0f * PI * HEADPAT_FLUTTER_FREQ_HZ * t + HEADPAT_FLUTTER_PHASE);

            rightTarget += headpatOffset + flutterRight;
            leftTarget += headpatOffset + flutterLeft;

            // Serial.printf(">headpat:%f\n", headpatIntensity);
        }

        // Listen — each ear independently tracks an idle directional target,
        // compensated for head rotation. The listen position lerps directly into each
        // ear's target at full intensity, suppressing contributions from other layers.
        // Right ear picks rightward targets; left ear leftward. Both stay in [0.2, 2.3]
        // to avoid high-current positions at the mechanical limits.
        {
            const float LISTEN_DEPTH = 0.8f;
            const float LISTEN_ATTACK_ALPHA = 0.08f;
            const float LISTEN_SUPPRESS_ALPHA = 0.005f;
            const float LISTEN_DECAY_ALPHA = 0.01f;
            const float LISTEN_RATE_HZ = 0.05f; // avg new events/s per ear (~1 per 20 s)
            const uint32_t LISTEN_HOLD_MIN_MS = 10000;
            const uint32_t LISTEN_HOLD_MAX_MS = 20000;
            const float LISTEN_SPAWN_THRESHOLD = 0.5f;    // max suppress to allow a new event
            const float LISTEN_SUPPRESS_THRESHOLD = 0.7f; // above this, active event fades

            const int32_t triggerThreshold = (int32_t)(LISTEN_RATE_HZ * dt * 65536.0f);
            const uint32_t now = millis();
            const float yawNow = imuState.yaw * DEG_TO_RAD;
            const float suppress = constrain(socialNorm + gaitNorm, 0.0f, 1.0f);

            // Right ear — rightward targets in [0.2, 2.3] rad
            if (!rightListenActive && suppress < LISTEN_SPAWN_THRESHOLD && random(65536) < triggerThreshold)
            {
                rightListenTarget = (float)random(20, 231) / 100.0f;
                rightListenRefYaw = yawNow;
                rightListenHoldUntil = now + LISTEN_HOLD_MIN_MS +
                                       (uint32_t)random(0L, (long)(LISTEN_HOLD_MAX_MS - LISTEN_HOLD_MIN_MS));
                rightListenActive = true;
            }
            {
                const bool inHold = rightListenActive && now < rightListenHoldUntil;
                // Hard threshold: during hold, intensity targets 1.0 while quiet and 0 when active.
                // Recovery from brief activity reuses the same fast attack alpha.
                const float desired = (inHold && suppress < LISTEN_SUPPRESS_THRESHOLD) ? 1.0f : 0.0f;
                if (rightListenActive && !inHold && rightListenIntensity < 0.01f)
                    rightListenActive = false;
                const float alpha = (desired > rightListenIntensity) ? LISTEN_ATTACK_ALPHA : inHold ? LISTEN_SUPPRESS_ALPHA
                                                                                                    : LISTEN_DECAY_ALPHA;
                rightListenIntensity += (desired - rightListenIntensity) * alpha;
                float headDelta = yawNow - rightListenRefYaw;
                headDelta = fmodf(headDelta + PI, 2.0f * PI) - PI;
                float listenPos = base - LISTEN_DEPTH * sinf(rightListenTarget - headDelta);
                listenPos = constrain(listenPos, 0.2f, 2.3f);
                rightTarget += (listenPos - rightTarget) * rightListenIntensity;
                // Serial.printf(">listenRPos:%f\n", listenPos);
                // Serial.printf(">listenRInt:%f\n", rightListenIntensity);
            }

            // Left ear — leftward targets in [-2.3, -0.2] rad; sign flipped so ear moves forward
            if (!leftListenActive && suppress < LISTEN_SPAWN_THRESHOLD && random(65536) < triggerThreshold)
            {
                leftListenTarget = -(float)random(20, 231) / 100.0f;
                leftListenRefYaw = yawNow;
                leftListenHoldUntil = now + LISTEN_HOLD_MIN_MS +
                                      (uint32_t)random(0L, (long)(LISTEN_HOLD_MAX_MS - LISTEN_HOLD_MIN_MS));
                leftListenActive = true;
            }
            {
                const bool inHold = leftListenActive && now < leftListenHoldUntil;
                const float desired = (inHold && suppress < LISTEN_SUPPRESS_THRESHOLD) ? 1.0f : 0.0f;
                if (leftListenActive && !inHold && leftListenIntensity < 0.01f)
                    leftListenActive = false;
                const float alpha = (desired > leftListenIntensity) ? LISTEN_ATTACK_ALPHA : inHold ? LISTEN_SUPPRESS_ALPHA
                                                                                                   : LISTEN_DECAY_ALPHA;
                leftListenIntensity += (desired - leftListenIntensity) * alpha;
                float headDelta = yawNow - leftListenRefYaw;
                headDelta = fmodf(headDelta + PI, 2.0f * PI) - PI;
                float listenPos = base + LISTEN_DEPTH * sinf(leftListenTarget - headDelta);
                listenPos = constrain(listenPos, 0.2f, 2.3f);
                leftTarget += (listenPos - leftTarget) * leftListenIntensity;
                // Serial.printf(">listenLPos:%f\n", listenPos);
                // Serial.printf(">listenLInt:%f\n", leftListenIntensity);
            }

            // Serial.printf(">listenSuppress:%f\n", suppress);
        }

        // Flicks — randomly triggered per ear, dampened sine wave transient.
        {
            const uint32_t FLICK_DURATION_MS = 200;
            const float FLICK_RATE_HZ = 0.3f;       // avg flicks/s per ear
            const float FLICK_AMPLITUDE_MIN = 1.0f; // rad
            const float FLICK_AMPLITUDE_MAX = 2.0f; // rad

            const int32_t flickTriggerThreshold = (int32_t)(FLICK_RATE_HZ * dt * 65536.0f);
            const uint32_t now = millis();
            if (random(65536) < flickTriggerThreshold && now - rightFlickStart > FLICK_DURATION_MS)
            {
                rightFlickStart = now;
                rightFlickAmplitude = FLICK_AMPLITUDE_MIN +
                                      (float)random(0, 101) / 100.0f * (FLICK_AMPLITUDE_MAX - FLICK_AMPLITUDE_MIN);
            }
            if (random(65536) < flickTriggerThreshold && now - leftFlickStart > FLICK_DURATION_MS)
            {
                leftFlickStart = now;
                leftFlickAmplitude = FLICK_AMPLITUDE_MIN +
                                     (float)random(0, 101) / 100.0f * (FLICK_AMPLITUDE_MAX - FLICK_AMPLITUDE_MIN);
            }

            float rightFlickOffset = 0.0f;
            float leftFlickOffset = 0.0f;
            if (now - rightFlickStart < FLICK_DURATION_MS)
            {
                float ft = (float)(now - rightFlickStart) / FLICK_DURATION_MS;
                rightFlickOffset = rightFlickAmplitude * expf(-4.0f * ft) * sinf(ft * PI * 2.0f);
            }
            if (now - leftFlickStart < FLICK_DURATION_MS)
            {
                float ft = (float)(now - leftFlickStart) / FLICK_DURATION_MS;
                leftFlickOffset = leftFlickAmplitude * expf(-4.0f * ft) * sinf(ft * PI * 2.0f);
            }

            rightTarget += rightFlickOffset;
            leftTarget += leftFlickOffset;
        }

        // ── Finalize ───────────────────────────────────────────────────────────

        Serial.printf(">RightTgt:%f\n", rightTarget);
        Serial.printf(">LeftTarget:%f\n", leftTarget);

        mot1->setPosition(constrain(rightTarget, 0.0f, MOTOR_THROW));
        mot2->setPosition(constrain(leftTarget, 0.0f, MOTOR_THROW));
    }
}
#pragma endregion

// FOC Task — motor control and I²t telemetry, runs on core 0 at TARGET_FOC_LOOP_HZ.
#pragma region FOC_Task
void FOC_Task(void *parameter)
{
    for (;;)
    {
#if TARGET_FOC_LOOP_HZ > 0
        static uint32_t nextUs = 0;
        if (nextUs == 0)
            nextUs = micros();
        while (micros() < nextUs)
        {
        }
        nextUs += 1000000UL / TARGET_FOC_LOOP_HZ;
#endif

        mot1->update();
        mot2->update();

        // Telemetry - I²t + FOC rate diagnostic at 1 Hz
        static uint32_t diagCount = 0;
        if (++diagCount >= TARGET_FOC_LOOP_HZ)
        {
            diagCount = 0;
            const auto &d1 = mot1->diagState();
            const auto &d2 = mot2->diagState();
            printf(
                "M1 %.0fHz Vq:%.2fV I:%.2fA accum:%.3f/%.3f trip:%.0f%% Vlim:%.2fV | "
                "M2 %.0fHz Vq:%.2fV I:%.2fA accum:%.3f/%.3f trip:%.0f%% Vlim:%.2fV\n",
                d1.loopFreqHz, d1.voltageQ, d1.currentEst, d1.i2tAccum, d1.i2tThreshold,
                d1.tripRatio * 100.f, d1.voltageLimit,
                d2.loopFreqHz, d2.voltageQ, d2.currentEst, d2.i2tAccum, d2.i2tThreshold,
                d2.tripRatio * 100.f, d2.voltageLimit);
        }
    }
}
#pragma endregion

// ============================================================================
#pragma region SETUP
// ============================================================================

// Hardware initialization and task dispatch
void setup()
{
    // FastLED first — needed for watchdog blink and init progress before anything else runs
    FastLED.addLeds<SK6812, RGB_LED_PIN, GRB>(onboard_led, 1);
    FastLED.setBrightness(50);

    // Watchdog reset lockout — blink magenta forever, require manual reset.
    // Something went badly wrong; don't attempt re-init automatically.
    esp_reset_reason_t resetReason = esp_reset_reason();
    if (resetReason == ESP_RST_TASK_WDT || resetReason == ESP_RST_INT_WDT)
    {
        for (;;)
        {
            onboard_led[0] = CRGB(200, 0, 200);
            FastLED.show();
            delay(250);
            onboard_led[0] = CRGB::Black;
            FastLED.show();
            delay(250);
        }
    }

    g_status = SystemStatus::STARTING;
    updateStatusLED();

    delay(2000);
    Serial.begin(115200);
    delay(100);

    Wire.begin(I2C_SDA, I2C_SCL);
    pinMode(BOOT_BUTTON_PIN, INPUT_PULLUP);

    // Distance Sensor — Short mode, 50 Hz continuous
    g_status = SystemStatus::INIT_TOF;
    updateStatusLED();
    distanceSensor.setTimeout(500);
    if (!distanceSensor.init())
    {
        g_status = SystemStatus::ERROR;
        updateStatusLED();
        Serial.println("Failed to detect and initialize VL53L1X sensor");
        while (1)
        {
        }
    }
    distanceSensor.setDistanceMode(VL53L1X::Short);   // better precision at close range (<1.3 m)
    distanceSensor.setMeasurementTimingBudget(20000); // 20 ms — minimum for Short mode
    distanceSensor.startContinuous(20);               // 50 Hz

    // IMU & Mahony
    g_status = SystemStatus::INIT_IMU;
    updateStatusLED();
    if (!imu.begin(Wire, MPU_ADDR,
                   CAL_XA, CAL_YA, CAL_ZA,
                   CAL_XG, CAL_YG, CAL_ZG,
                   SAMPLE_HZ,
                   0.0f)) // yawDecay: surprisingly not necessary
    {
        g_status = SystemStatus::ERROR;
        updateStatusLED();
        Serial.println("IMU not found — check wiring");
        while (true)
        {
            delay(10);
        }
    }

    // SimpleFOC
    SimpleFOCDebug::enable(&Serial);
    SPI.begin(SENSOR_SPI_CLK, SENSOR_SPI_MISO, SENSOR_SPI_MOSI);

    // Command LPF time constant: Tf = 1/(2π·fc), derived from COMMAND_LPF_HZ.
    const float commandLpfTf = 1.0f / (2.0f * PI * COMMAND_LPF_HZ);

    // Motor 1 — right ear (Board 1)
    EarMotorConfig cfg1;
    cfg1.forwardAngle = 6.1f;
    cfg1.backAngle = 3.6f;
    cfg1.zeroElectricAngle = 4.44;
    cfg1.sensorDirection = Direction::CW;
    cfg1.commandLpfTf = commandLpfTf;

    // Motor 2 — left ear (Board 2)
    EarMotorConfig cfg2;
    cfg2.forwardAngle = 1.1f;
    cfg2.backAngle = 3.6f;
    cfg2.zeroElectricAngle = 0.57;
    cfg2.sensorDirection = Direction::CW;
    cfg2.commandLpfTf = commandLpfTf;

    mot1 = new EarMotor(SPI, B1_SENSOR_CS_PIN,
                        B1_MOTOR_PWM_A, B1_MOTOR_PWM_B, B1_MOTOR_PWM_C, B1_MOTOR_ENABLE, cfg1);
    mot2 = new EarMotor(SPI, B2_SENSOR_CS_PIN,
                        B2_MOTOR_PWM_A, B2_MOTOR_PWM_B, B2_MOTOR_PWM_C, B2_MOTOR_ENABLE, cfg2);

    g_status = SystemStatus::INIT_MOTOR_1;
    updateStatusLED();
    if (mot1->init() != EarMotor::InitResult::OK)
    {
        g_status = SystemStatus::ERROR;
        updateStatusLED();
        Serial.println("Motor 1 init failed");
        while (true)
        {
            delay(10);
        }
    }

    g_status = SystemStatus::INIT_MOTOR_2;
    updateStatusLED();
    if (mot2->init() != EarMotor::InitResult::OK)
    {
        g_status = SystemStatus::ERROR;
        updateStatusLED();
        Serial.println("Motor 2 init failed");
        while (true)
        {
            delay(10);
        }
    }

    g_status = SystemStatus::RUNNING;
    updateStatusLED();

    // ============================================================================
    // Task Dispatch
    // ============================================================================

    // FOC on core 0 (non-Arduino core) — busy-wait timing, no idle task
    xTaskCreatePinnedToCore(
        FOC_Task,
        "FOC_Task",
        4096,
        NULL,
        1,
        NULL,
        !ARDUINO_RUNNING_CORE);

    // Core 0 is fully dedicated to FOC_Task; remove its idle task from Task Watch Dog Timer.
    esp_task_wdt_delete(xTaskGetIdleTaskHandleForCore(!ARDUINO_RUNNING_CORE));

    // Animation + sensors on core 1 — uses vTaskDelayUntil, plays nice with idle task
    xTaskCreatePinnedToCore(
        Anim_Task,
        "Anim_Task",
        8192,
        NULL,
        1,
        NULL,
        ARDUINO_RUNNING_CORE);

    // Status LED on core 1 — low rate, updates g_status-driven color at 10 Hz
    xTaskCreatePinnedToCore(
        LED_Task,
        "LED_Task",
        2048,
        NULL,
        1,
        NULL,
        ARDUINO_RUNNING_CORE);
}

#pragma endregion

// ============================================================================
#pragma region MAIN LOOP
// ============================================================================

void loop()
{
}

#pragma endregion
