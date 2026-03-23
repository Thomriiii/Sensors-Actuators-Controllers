// AS5600 simple serial output (raw angle and degrees)
// I2C address: 0x36
// Wiring: VCC->5V/3.3V, GND->GND, SDA->A4 (UNO), SCL->A5 (UNO)

#include <Wire.h>

const uint8_t AS5600_ADDR = 0x36;
const uint8_t RAW_ANGLE_MSB = 0x0C;

uint16_t readRawAngle() {
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(RAW_ANGLE_MSB);
  if (Wire.endTransmission(false) != 0) {
    return 0xFFFF; // error
  }

  Wire.requestFrom(AS5600_ADDR, (uint8_t)2);
  if (Wire.available() < 2) {
    return 0xFFFF; // error
  }

  uint8_t msb = Wire.read();
  uint8_t lsb = Wire.read();
  return ((uint16_t)msb << 8) | lsb; // 12-bit value in bits [11:0]
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  delay(100);
  Serial.println("AS5600 raw angle (0-4095) and degrees");
}

void loop() {
  uint16_t raw = readRawAngle();
  if (raw == 0xFFFF) {
    Serial.println("I2C read error");
  } else {
    raw &= 0x0FFF; // 12-bit mask
    float degrees = (raw * 360.0f) / 4096.0f;
    Serial.print("raw=");
    Serial.print(raw);
    Serial.print("  deg=");
    Serial.println(degrees, 2);
  }
  delay(100);
}
