#include <Adafruit_NeoPixel.h>

#define LED_PIN 6
#define NUM_LEDS 1
#define LDR_PIN A1

Adafruit_NeoPixel strip(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

const int STEP_OUTPUT = 20;
const int STEP_DELAY_SAMPLES = 35;
int sampleIndex = 0;

int readSensorAverage(int samples) {
  long acc = 0;
  for (int i = 0; i < samples; i++) {
    acc += analogRead(LDR_PIN);
  }
  return (int)(acc / samples);
}

void applyOutput(int brightness) {
  int limited = constrain(brightness, 0, 255);
  strip.setPixelColor(0, strip.Color(limited, limited, limited));
  strip.show();
}

float measureContribution(int commandedOutput) {
  applyOutput(0);
  delay(20);
  int ambient = readSensorAverage(8);

  applyOutput(commandedOutput);
  delay(30);
  int total = readSensorAverage(8);

  int contribution = total - ambient;
  if (contribution < 0) {
    contribution = 0;
  }
  return (float)contribution;
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    // Wait for the host to open the serial port before starting the test.
  }
  strip.begin();
  strip.show();
}

void loop() {
  int output = sampleIndex < STEP_DELAY_SAMPLES ? 0 : STEP_OUTPUT;
  float sensor = measureContribution(output);

  Serial.print(sensor, 2);
  Serial.print(",");
  Serial.println(output);

  sampleIndex++;
  delay(120);
}
