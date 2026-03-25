#include <Adafruit_NeoPixel.h>
#include <math.h>

#define LED_PIN 6
#define NUM_LEDS 1
#define LDR_PIN A1

Adafruit_NeoPixel strip(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

// PID gains tuned for a slow optical plant with dual-sampling feedback.
float Kp = 0.38f;
float Ki = 0.16f;
float Kd = 0.10f;

// Timing and filtering.
const unsigned long CONTROL_PERIOD_MS = 220;
const unsigned long AMBIENT_SETTLE_MS = 20;
const unsigned long LED_SETTLE_MS = 30;
const unsigned long AMBIENT_REFRESH_MS = 3000;
const float FILTER_ALPHA = 0.18f;
const float DERIV_ALPHA = 0.12f;
const float ERROR_DEADBAND = 3.0f;
const float AMBIENT_ESTIMATE_ALPHA = 0.25f;

// Output protection.
const float OUTPUT_MIN = 0.0f;
const float OUTPUT_MAX = 255.0f;
const float OUTPUT_STEP_LIMIT = 8.0f;
const float INTEGRAL_MIN = -90.0f;
const float INTEGRAL_MAX = 90.0f;
const float INTEGRAL_ACTIVE_BAND = 70.0f;
const float INTEGRAL_BLEED = 0.55f;

// Calibration targets.
const int CALIBRATION_SAMPLES = 8;
const float SETPOINT_FRACTION = 0.60f;

float setpoint = 0.0f;
float ledContributionMax = 0.0f;
float ambientEstimate = 0.0f;

float pvFiltered = 0.0f;
float prevPvFiltered = 0.0f;
float prevError = 0.0f;
float dFiltered = 0.0f;
float integral = 0.0f;
float output = 0.0f;

unsigned long lastControlMs = 0;
unsigned long lastAmbientRefreshMs = 0;

int readSensorAverage(int samples) {
  long acc = 0;
  for (int i = 0; i < samples; i++) {
    acc += analogRead(LDR_PIN);
  }
  return (int)(acc / samples);
}

void applyOutput(float value) {
  int b = constrain((int)(value + 0.5f), 0, 255);
  strip.setPixelColor(0, strip.Color(b, b, b));
  strip.show();
}

float sampleAmbient(float restoreOutput) {
  applyOutput(0.0f);
  delay(AMBIENT_SETTLE_MS);
  float ambient = (float)readSensorAverage(8);
  applyOutput(restoreOutput);
  return ambient;
}

float sampleTotal(float commandedOutput, bool allowSettle) {
  applyOutput(commandedOutput);
  if (allowSettle) {
    delay(LED_SETTLE_MS);
  }
  return (float)readSensorAverage(8);
}

float measureContribution(float commandedOutput) {
  float ambient = sampleAmbient(commandedOutput);
  float total = sampleTotal(commandedOutput, true);

  float contribution = total - ambient;
  if (contribution < 0.0f) {
    contribution = 0.0f;
  }
  return contribution;
}

void calibrate() {
  Serial.println("Calibrating...");

  float contributionSum = 0.0f;
  for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
    contributionSum += measureContribution(255.0f);
    delay(40);
  }

  ledContributionMax = contributionSum / CALIBRATION_SAMPLES;
  if (ledContributionMax < 20.0f) {
    ledContributionMax = 20.0f;
  }

  setpoint = ledContributionMax * SETPOINT_FRACTION;
  ambientEstimate = sampleAmbient(0.0f);

  pvFiltered = 0.0f;
  prevPvFiltered = pvFiltered;
  prevError = 0.0f;
  dFiltered = 0.0f;
  integral = 0.0f;
  output = 0.0f;
  applyOutput(output);

  Serial.print("MaxContribution: ");
  Serial.println(ledContributionMax, 2);
  Serial.print("Setpoint: ");
  Serial.println(setpoint, 2);
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    // Wait for native USB serial.
  }

  strip.begin();
  strip.show();

  calibrate();
  lastControlMs = millis();
  lastAmbientRefreshMs = millis();

  Serial.println("raw,filtered,setpoint,error,output");
}

void loop() {
  unsigned long now = millis();
  unsigned long elapsedMs = now - lastControlMs;
  if (elapsedMs < CONTROL_PERIOD_MS) {
    delay(CONTROL_PERIOD_MS - elapsedMs);
    now = millis();
    elapsedMs = now - lastControlMs;
  }
  lastControlMs = now;

  float dt = elapsedMs / 1000.0f;
  if (dt <= 0.0f) {
    dt = CONTROL_PERIOD_MS / 1000.0f;
  }

  float total = sampleTotal(output, false);
  float raw = total - ambientEstimate;
  if (raw < 0.0f) {
    raw = 0.0f;
  }
  pvFiltered += FILTER_ALPHA * (raw - pvFiltered);

  float error = setpoint - pvFiltered;
  if (fabs(error) < ERROR_DEADBAND) {
    error = 0.0f;
  }

  float dMeas = (pvFiltered - prevPvFiltered) / dt;
  prevPvFiltered = pvFiltered;
  dFiltered += DERIV_ALPHA * (dMeas - dFiltered);

  float pTerm = Kp * error;
  float dTerm = -Kd * dFiltered;

  bool crossedSetpoint = ((error > 0.0f && prevError < 0.0f) ||
                          (error < 0.0f && prevError > 0.0f));
  if (crossedSetpoint) {
    integral *= INTEGRAL_BLEED;
  }

  float unsatWithoutIntegral = pTerm + dTerm + integral;
  bool pushingHighLimit = (unsatWithoutIntegral >= OUTPUT_MAX && error > 0.0f);
  bool pushingLowLimit = (unsatWithoutIntegral <= OUTPUT_MIN && error < 0.0f);
  bool allowIntegral = fabs(error) <= INTEGRAL_ACTIVE_BAND;
  if (allowIntegral && !pushingHighLimit && !pushingLowLimit) {
    integral += Ki * error * dt;
    integral = constrain(integral, INTEGRAL_MIN, INTEGRAL_MAX);
  }

  float command = pTerm + integral + dTerm;
  command = constrain(command, OUTPUT_MIN, OUTPUT_MAX);

  if (command > output) {
    command = min(command, output + OUTPUT_STEP_LIMIT);
  } else {
    command = max(command, output - OUTPUT_STEP_LIMIT);
  }

  output = constrain(command, OUTPUT_MIN, OUTPUT_MAX);
  applyOutput(output);

  Serial.print(raw, 2);
  Serial.print(",");
  Serial.print(pvFiltered, 2);
  Serial.print(",");
  Serial.print(setpoint, 2);
  Serial.print(",");
  Serial.print(error, 2);
  Serial.print(",");
  Serial.println(output, 2);

  if (now - lastAmbientRefreshMs >= AMBIENT_REFRESH_MS) {
    float ambientSample = sampleAmbient(output);
    ambientEstimate += AMBIENT_ESTIMATE_ALPHA * (ambientSample - ambientEstimate);
    delay(LED_SETTLE_MS);
    lastAmbientRefreshMs = millis();
  }

  prevError = error;
}
