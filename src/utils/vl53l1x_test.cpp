/**
 * VL53L1X Time-of-Flight Sensor Test Utility
 *
 * Tests the VL53L1X ToF distance sensor with visual feedback.
 * Reads distance measurements and displays them with a color-coded LED.
 *
 * Color coding (easy to read):
 * - Blue: Very close (0-100mm)
 * - Cyan: Close (100-300mm)
 * - Green: Medium (300-600mm)
 * - Yellow: Far (600-1000mm)
 * - Red: Very far (1000-2000mm)
 * - Magenta: Out of range (>2000mm)
 */

#include <Arduino.h>
#include <Wire.h>
#include <VL53L1X.h>
#include <FastLED.h>

#define I2C_SDA 17
#define I2C_SCL 18
#define RGB_LED_PIN 8

VL53L1X sensor;
CRGB onboard_led[1];

// Map distance to color with easy-to-read gradient
CRGB distanceToColor(uint16_t distance)
{
  if (distance < 10)
  {
    // Extremely close: Deep purple (almost touching)
    return CRGB(128, 0, 128);
  }
  else if (distance < 200)
  {
    // High resolution close range: Purple through Blue to Cyan
    // 10-70mm: Purple to Blue
    // 70-130mm: Blue to Cyan
    // 130-200mm: Cyan to light cyan
    if (distance < 70)
    {
      uint8_t blend = map(distance, 10, 70, 0, 255);
      return CRGB(128 - blend / 2, 0, 128 + blend / 2); // Purple to Blue
    }
    else if (distance < 130)
    {
      uint8_t blend = map(distance, 70, 130, 0, 255);
      return CRGB(0, blend, 255); // Blue to Cyan
    }
    else
    {
      uint8_t blend = map(distance, 130, 200, 0, 255);
      return CRGB(0, 255, 255 - blend / 3); // Cyan to light cyan
    }
  }
  else if (distance < 300)
  {
    // Close: Cyan to Green gradient
    uint8_t blend = map(distance, 200, 300, 0, 255);
    return CRGB(0, 255, 170 - blend * 170 / 255);
  }
  else if (distance < 600)
  {
    // Medium: Green gradient
    uint8_t blend = map(distance, 300, 600, 0, 255);
    return CRGB(0, 255, 0);
  }
  else if (distance < 1000)
  {
    // Far: Green to Yellow gradient
    uint8_t blend = map(distance, 600, 1000, 0, 255);
    return CRGB(blend, 255, 0);
  }
  else if (distance < 2000)
  {
    // Very far: Yellow to Red gradient
    uint8_t blend = map(distance, 1000, 2000, 0, 255);
    return CRGB(255, 255 - blend, 0);
  }
  else
  {
    // Out of range: Magenta
    return CRGB::Magenta;
  }
}

void setup()
{
  delay(2000);
  Serial.begin(115200);
  Serial.println("\nVL53L1X ToF Sensor Test with LED Feedback");

  // Initialize LED
  FastLED.addLeds<SK6812, RGB_LED_PIN>(onboard_led, 1);
  FastLED.setBrightness(50); // Not too bright
  onboard_led[0] = CRGB::Black;
  FastLED.show();

  // Initialize I2C and sensor
  Wire.begin(I2C_SDA, I2C_SCL, 100000);

  sensor.setTimeout(500);
  if (!sensor.init())
  {
    Serial.println("Failed to detect and initialize VL53L1X sensor!");
    // Flash red on error
    while (1)
    {
      onboard_led[0] = CRGB::Red;
      FastLED.show();
      delay(200);
      onboard_led[0] = CRGB::Black;
      FastLED.show();
      delay(200);
    }
  }

  sensor.setDistanceMode(VL53L1X::Long);
  sensor.setMeasurementTimingBudget(50000);
  sensor.startContinuous(50);

  Serial.println("VL53L1X initialized successfully!");
  Serial.println("Distance ranges (high resolution close range):");
  Serial.println("  <10mm: Deep Purple");
  Serial.println("  10-70mm: Purple to Blue");
  Serial.println("  70-130mm: Blue to Cyan");
  Serial.println("  130-200mm: Cyan to Light Cyan");
  Serial.println("  200-300mm: Cyan to Green");
  Serial.println("  300-600mm: Green");
  Serial.println("  600-1000mm: Yellow");
  Serial.println("  1000-2000mm: Red");
  Serial.println("  >2000mm: Magenta");
}

void loop()
{
  uint16_t distance = sensor.read();

  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.print(" mm");

  if (sensor.timeoutOccurred())
  {
    Serial.print(" (TIMEOUT)");
    onboard_led[0] = CRGB::White; // White for timeout
  }
  else
  {
    onboard_led[0] = distanceToColor(distance);
  }

  FastLED.show();
  Serial.println();

  delay(100);
}
