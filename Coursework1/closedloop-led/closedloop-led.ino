#include <Adafruit_NeoPixel.h>

#define LED_PIN 6
#define NUM_LEDS 1
#define LDR_PIN A1

Adafruit_NeoPixel strip(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

// ===== PID PARAMETERS =====
float Kp = 0.030f;
float Ki = 0.001f;
float Kd = 0.014f;
float setpoint = 500.0f;

// ===== CONTROL SETTINGS =====
const unsigned long SAMPLE_MS = 100;
const float DT = SAMPLE_MS / 1000.0f;
const float SENSOR_ALPHA = 0.18f;
const float DERIV_ALPHA = 0.25f;
const float ERROR_DEADBAND = 1.0f;
const float MAX_STEP_UP = 2.5f;
const float MAX_STEP_DOWN = 4.0f;
const float INTEGRAL_MIN = -80.0f;
const float INTEGRAL_MAX = 80.0f;
const float INTEGRAL_ACTIVE_BAND = 80.0f;
const float INTEGRAL_BLEED_ON_CROSS = 0.1f;
const int BASE_OUTPUT_DEFAULT = 28;
const int CALIBRATION_OUTPUT = 32;

// ===== PID STATE =====
float pvFiltered = 0.0f;
float prevPvFiltered = 0.0f;
float prevError = 0.0f;
float dMeasFiltered = 0.0f;
float integral = 0.0f;
float output = 0.0f;
unsigned long lastSampleMs = 0;
int plantSign = 1;
float baseOutput = BASE_OUTPUT_DEFAULT;
bool csvHeaderPrinted = false;

int maxOutput = 255;
int minOutput = 0;

String serialLine;

int readLdrAveraged() {
  long acc = 0;
  for (int i = 0; i < 12; i++) {
    acc += analogRead(LDR_PIN);
  }
  return (int)(acc / 12);
}

int measureSensor(int samples, int delayMs) {
  long acc = 0;
  for (int i = 0; i < samples; i++) {
    acc += readLdrAveraged();
    delay(delayMs);
  }
  return (int)(acc / samples);
}

void applyOutput(float value) {
  int brightness = constrain((int)(value + 0.5f), minOutput, maxOutput);
  strip.setPixelColor(0, strip.Color(brightness, brightness, brightness));
  strip.show();
}

void calibrateSystem() {
  applyOutput(0.0f);
  delay(250);
  int ambient = measureSensor(20, 10);

  applyOutput(baseOutput);
  delay(250);
  int baseLit = measureSensor(20, 10);

  applyOutput(CALIBRATION_OUTPUT);
  delay(250);
  int probeLit = measureSensor(20, 10);

  plantSign = (probeLit >= ambient) ? 1 : -1;
  setpoint = constrain((float)baseLit, 0.0f, 1023.0f);
  applyOutput(baseOutput);
  delay(150);

  Serial.print("Calibration ambient=");
  Serial.print(ambient);
  Serial.print(" baseLit=");
  Serial.print(baseLit);
  Serial.print(" probeLit=");
  Serial.print(probeLit);
  Serial.print(" sign=");
  Serial.print(plantSign);
  Serial.print(" baseOutput=");
  Serial.print(baseOutput, 1);
  Serial.print(" setpoint=");
  Serial.println(setpoint, 1);
}

void resetController() {
  int sensor = readLdrAveraged();
  pvFiltered = sensor;
  prevPvFiltered = sensor;
  prevError = 0.0f;
  dMeasFiltered = 0.0f;
  integral = 0.0f;
  output = baseOutput;
  applyOutput(output);
  lastSampleMs = millis();
}

void printHelp() {
  Serial.println("=== PID SERIAL CONTROL ===");
  Serial.println("p<value> : Set Kp");
  Serial.println("i<value> : Set Ki");
  Serial.println("d<value> : Set Kd");
  Serial.println("s<value> : Set Setpoint");
  Serial.println("b<value> : Set base output");
  Serial.println("r        : Reset controller");
  Serial.println("c        : Recalibrate system");
  Serial.println("?        : Show values");
  Serial.println("==========================");
}

void printValues() {
  Serial.println("Current PID values:");
  Serial.print("Kp: "); Serial.println(Kp, 4);
  Serial.print("Ki: "); Serial.println(Ki, 4);
  Serial.print("Kd: "); Serial.println(Kd, 4);
  Serial.print("Setpoint: "); Serial.println(setpoint, 1);
  Serial.print("Base output: "); Serial.println(baseOutput, 1);
}

void handleSerial() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c != '\n') {
      serialLine += c;
      continue;
    }

    serialLine.trim();
    if (serialLine.length() == 0) {
      serialLine = "";
      continue;
    }

    char command = serialLine.charAt(0);
    float value = serialLine.substring(1).toFloat();

    switch (command) {
      case 'p':
        Kp = max(0.0f, value);
        Serial.println("Updated Kp");
        break;

      case 'i':
        Ki = max(0.0f, value);
        Serial.println("Updated Ki");
        break;

      case 'd':
        Kd = max(0.0f, value);
        Serial.println("Updated Kd");
        break;

      case 's':
        setpoint = constrain(value, 0.0f, 1023.0f);
        Serial.println("Updated Setpoint");
        break;

      case 'b':
        baseOutput = constrain(value, (float)minOutput, (float)maxOutput);
        Serial.println("Updated Base Output");
        break;

      case 'r':
        resetController();
        Serial.println("Controller reset");
        break;

      case 'c':
        calibrateSystem();
        resetController();
        Serial.println("Calibration complete");
        break;

      case '?':
        printValues();
        break;

      default:
        Serial.println("Unknown command");
        printHelp();
        break;
    }

    serialLine = "";
  }
}

void setup() {
  pinMode(LDR_PIN, INPUT);

  strip.begin();
  strip.clear();
  strip.show();

  Serial.begin(115200);
  delay(1000);

  printHelp();
  calibrateSystem();
  resetController();
}

void loop() {
  handleSerial();

  if (!csvHeaderPrinted) {
    Serial.println("sensor,pvFilt,setpoint,error,pTerm,iTerm,dTerm,output");
    csvHeaderPrinted = true;
  }

  unsigned long now = millis();
  if ((now - lastSampleMs) < SAMPLE_MS) return;
  lastSampleMs += SAMPLE_MS;

  int sensor = readLdrAveraged();
  pvFiltered += SENSOR_ALPHA * ((float)sensor - pvFiltered);

  float error = plantSign * (setpoint - pvFiltered);
  if (fabs(error) < ERROR_DEADBAND) {
    error = 0.0f;
  }

  if ((error > 0.0f && prevError < 0.0f) || (error < 0.0f && prevError > 0.0f)) {
    integral *= INTEGRAL_BLEED_ON_CROSS;
  }

  float dMeas = (pvFiltered - prevPvFiltered) / DT;
  prevPvFiltered = pvFiltered;
  dMeasFiltered += DERIV_ALPHA * (dMeas - dMeasFiltered);

  float pTerm = Kp * error;
  float dTerm = -Kd * dMeasFiltered;

  float unclamped = baseOutput + pTerm + integral + dTerm;
  bool saturatingHigh = unclamped > maxOutput && error > 0.0f;
  bool saturatingLow = unclamped < minOutput && error < 0.0f;
  bool errorIsModerate = fabs(error) < INTEGRAL_ACTIVE_BAND;
  if (!(saturatingHigh || saturatingLow) && errorIsModerate) {
    integral += Ki * error * DT;
    integral = constrain(integral, INTEGRAL_MIN, INTEGRAL_MAX);
  }

  float commanded = baseOutput + pTerm + integral + dTerm;
  commanded = constrain(commanded, (float)minOutput, (float)maxOutput);
  if (commanded > output) {
    commanded = min(commanded, output + MAX_STEP_UP);
  } else {
    commanded = max(commanded, output - MAX_STEP_DOWN);
  }
  output = commanded;

  applyOutput(output);

  Serial.print(sensor);
  Serial.print(",");
  Serial.print(pvFiltered, 2);
  Serial.print(",");
  Serial.print(setpoint, 1);
  Serial.print(",");
  Serial.print(error, 2);
  Serial.print(",");
  Serial.print(pTerm, 3);
  Serial.print(",");
  Serial.print(integral, 3);
  Serial.print(",");
  Serial.print(dTerm, 3);
  Serial.print(",");
  Serial.println(output, 2);

  prevError = error;
}
