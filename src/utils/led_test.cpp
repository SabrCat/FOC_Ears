#include "Arduino.h"
#include "FastLED.h"

CRGB onboard_led[1];

#define PIN_RGB_LED 8

void setup() {
  FastLED.addLeds<WS2812B, PIN_RGB_LED, GRB>(onboard_led, 1);
  FastLED.setBrightness(255 / 10);
  
  Serial.begin(115200);
}

void loop() {
  Serial.println("Hi I'm working");
  onboard_led[0] = CRGB::Red; FastLED.show(); delay(500);
  onboard_led[0] = CRGB::Blue; FastLED.show(); delay(500);        
}