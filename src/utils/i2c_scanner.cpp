/**
 * I2C Scanner Utility
 * 
 * Scans all I2C addresses and reports which devices are found.
 * Useful for debugging sensor connections.
 */

#include <Arduino.h>
// #include <Adafruit_Sensor.h>
#include <Wire.h>
 
#define I2C_SDA 17
#define I2C_SCL 18



void setup() {
  Wire.begin(I2C_SDA, I2C_SCL, 400000);
  Serial.begin(115200);
  Serial.println("\nI2C Scanner");
}
 
void loop() {
  byte error, address;
  int nDevices;
  Serial.println("Scanning...");
  nDevices = 0;
  for(address = 1; address < 127; address++ ) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address<16) {
        Serial.print("0");
      }
      Serial.println(address,HEX);
      nDevices++;
    }
    else if (error==4) {
      Serial.print("Unknown error at address 0x");
      if (address<16) {
        Serial.print("0");
      }
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0) {
    Serial.println("No I2C devices found\n");
  }
  else {
    Serial.println("done\n");
  }
  delay(5000);          
}