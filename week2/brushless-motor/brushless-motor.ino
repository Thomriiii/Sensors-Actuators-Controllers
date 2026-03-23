// QWinOut 30A Brushless ESC control on pin 5
// Uses standard servo PWM: 1000-2000us, 50Hz

#include <Servo.h>

const uint8_t ESC_PIN = 5;

Servo esc;

const int THROTTLE_MIN_US = 1000; // stop
const int THROTTLE_MAX_US = 2000; // full
const int THROTTLE_ARM_US = 1000; // arming signal

void setup() {
  Serial.begin(115200);

  esc.attach(ESC_PIN, THROTTLE_MIN_US, THROTTLE_MAX_US);

  // Arm ESC: send minimum throttle for a few seconds
  esc.writeMicroseconds(THROTTLE_ARM_US);
  Serial.println("Arming ESC (min throttle)");
  delay(3000);
}

void loop() {
  // Simple demo: ramp up, hold, ramp down
  for (int us = THROTTLE_MIN_US; us <= 1600; us += 5) {
    esc.writeMicroseconds(us);
    delay(20);
  }

  delay(1000);

  for (int us = 1600; us >= THROTTLE_MIN_US; us -= 5) {
    esc.writeMicroseconds(us);
    delay(20);
  }

  delay(1500);
}
