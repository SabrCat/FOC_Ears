/**
 * FOC Ears Project - Main Entry Point
 * 
 * This file acts as a dispatcher to different application modes.
 * The actual implementation is determined by the PlatformIO environment selected.
 * 
 * Available environments (switch in PlatformIO toolbar):
 * - foc_ears: Main FOC ears application with IMU control
 * - i2c_scanner: I2C bus scanner utility
 * - motor_test: Motor testing and calibration
 * - sensor_test: Sensor debugging utility
 * 
 * To add a new mode:
 * 1. Create a new .cpp file in src/apps/ or src/utils/
 * 2. Add a new environment in platformio.ini with appropriate build_flags
 * 3. Add the corresponding #elif block below
 */

// Main application implementations
#if defined(USE_FOC_EARS)
    #include "apps_2/foc_ears_main.cpp"

// Utility/debugging tools
#elif defined(USE_I2C_SCANNER)
    #include "utils/i2c_scanner.cpp"
#elif defined(USE_MOTOR_TEST)
    #include "utils/motor_test.cpp"
#elif defined(USE_SENSOR_TEST)
    #include "utils/sensor_test.cpp"
#elif defined(USE_BNO055_TEST)
    #include "utils/bno055_test.cpp"
#elif defined(USE_LED_TEST)
    #include "utils/led_test.cpp"
#elif defined(USE_VL53L1X_TEST)
    #include "utils/vl53l1x_test.cpp"
#elif defined(USE_MPU6050_TEST)
    #include "utils/mpu6050_test.cpp"
#elif defined(USE_OPENLOOP_TEST)
    #include "utils/openloop_test.cpp"
#elif defined(USE_CLOSEDLOOP_TEST)
    #include "utils/closedloop_test.cpp"
#elif defined(USE_CALIBRATION_TEST)
    #include "utils/calibration_test.cpp"

#else
    #error "No application mode selected! Please select a PlatformIO environment."
#endif