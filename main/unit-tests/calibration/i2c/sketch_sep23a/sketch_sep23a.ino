#include <Arduino.h>
#include <Wire.h>

void setup() {
  Serial.begin(115200);

  // Default ESP32 I2C pins:
  // SDA = 21
  // SCL = 22
  Wire.begin(21, 22);

  Serial.println();
  Serial.println("I2C Scanner");
}

void loop() {
  byte error;
  int devices = 0;

  Serial.println("Scanning...");

  for (byte address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("I2C device found at 0x");

      if (address < 16)
        Serial.print("0");

      Serial.println(address, HEX);
      devices++;
    }
    else if (error == 4) {
      Serial.print("Unknown error at 0x");

      if (address < 16)
        Serial.print("0");

      Serial.println(address, HEX);
    }
  }

  if (devices == 0)
    Serial.println("No I2C devices found.");
  else {
    Serial.print("Found ");
    Serial.print(devices);
    Serial.println(" device(s).");
  }

  Serial.println("------------------------");

  delay(100);
}