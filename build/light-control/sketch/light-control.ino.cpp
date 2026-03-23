#include <Arduino.h>
#line 1 "C:\\Users\\henry\\Documents\\Sensors-Actuators-Controllers\\week1\\light-control\\light-control.ino"
// PID brightness control (WS2812 LED + LDR sensor)
// Designed for stable visible output with quick disturbance recovery.

#include <Adafruit_NeoPixel.h>

const int PIN_LED_DIN = 5;
const int PIN_LDR = A0;
const int LED_COUNT = 1;

Adafruit_NeoPixel strip(LED_COUNT, PIN_LED_DIN, NEO_GRB + NEO_KHZ800);

// Control loop timing.
const unsigned long SAMPLE_MS = 20;     // 50 Hz loop.
const float DT = SAMPLE_MS / 1000.0f;

// Filtering and stability controls.
const float SENSOR_ALPHA = 0.12f;       // Sensor LPF (higher = faster, lower = smoother).
const float DERIV_ALPHA = 0.25f;        // Derivative LPF.
const float ERROR_DEADBAND = 2.0f;      // Ignore tiny error to reduce visible shimmer.
const float MAX_STEP = 2.0f;            // Max brightness step per loop.

// PID gains (sensible starting point for LDR + LED loop).
float Kp = 0.080f;
float Ki = 0.020f;
float Kd = 0.010f;

// Runtime setpoint and baseline.
int midpoint = 620;                     // Ambient midpoint estimate.
int setpoint = 680;                     // Target around midpoint + offset.
int plantSign = 1;                      // +1: LED up -> LDR up, -1: LED up -> LDR down.

// Controller state.
float pvFilt = 0.0f;
float dMeasFilt = 0.0f;
float iTerm = 0.0f;
float prevPv = 0.0f;
float outPrev = 0.0f;
unsigned long lastControlMs = 0;

// Serial interface.
String serialLine;
const bool PLOTTER_MODE = false;         // true: CSV only for plotter labels/data.

#line 44 "C:\\Users\\henry\\Documents\\Sensors-Actuators-Controllers\\week1\\light-control\\light-control.ino"
int readLdrAveraged();
#line 52 "C:\\Users\\henry\\Documents\\Sensors-Actuators-Controllers\\week1\\light-control\\light-control.ino"
void setWhiteBrightness(int brightness);
#line 58 "C:\\Users\\henry\\Documents\\Sensors-Actuators-Controllers\\week1\\light-control\\light-control.ino"
void autoCalibrateMidpoint();
#line 92 "C:\\Users\\henry\\Documents\\Sensors-Actuators-Controllers\\week1\\light-control\\light-control.ino"
void printStatus();
#line 101 "C:\\Users\\henry\\Documents\\Sensors-Actuators-Controllers\\week1\\light-control\\light-control.ino"
void handleSerial();
#line 141 "C:\\Users\\henry\\Documents\\Sensors-Actuators-Controllers\\week1\\light-control\\light-control.ino"
void setup();
#line 163 "C:\\Users\\henry\\Documents\\Sensors-Actuators-Controllers\\week1\\light-control\\light-control.ino"
void loop();
#line 44 "C:\\Users\\henry\\Documents\\Sensors-Actuators-Controllers\\week1\\light-control\\light-control.ino"
int readLdrAveraged() {
  int acc = 0;
  for (int i = 0; i < 8; i++) {
    acc += analogRead(PIN_LDR);
  }
  return acc / 8;
}

void setWhiteBrightness(int brightness) {
  brightness = constrain(brightness, 0, 255);
  strip.setPixelColor(0, strip.Color(brightness, brightness, brightness));
  strip.show();
}

void autoCalibrateMidpoint() {
  // LED off while measuring ambient so setpoint starts in a sensible range.
  setWhiteBrightness(0);
  delay(200);
  long sum = 0;
  const int n = 100;
  for (int i = 0; i < n; i++) {
    sum += readLdrAveraged();
    delay(5);
  }
  int ambient = (int)(sum / n);

  // Detect process direction using a small LED step.
  setWhiteBrightness(80);
  delay(180);
  long litSum = 0;
  for (int i = 0; i < 40; i++) {
    litSum += readLdrAveraged();
    delay(4);
  }
  int lit = (int)(litSum / 40);
  setWhiteBrightness(0);

  plantSign = (lit > ambient) ? 1 : -1;

  // Choose setpoint in the controllable direction from ambient.
  const int offset = 120;
  midpoint = ambient;
  setpoint = constrain(ambient + (plantSign * offset), 10, 1010);

  pvFilt = ambient;
  prevPv = ambient;
}

void printStatus() {
  Serial.print("Kp="); Serial.print(Kp, 4);
  Serial.print(" Ki="); Serial.print(Ki, 4);
  Serial.print(" Kd="); Serial.print(Kd, 4);
  Serial.print(" S="); Serial.print(plantSign);
  Serial.print(" MID="); Serial.print(midpoint);
  Serial.print(" SP="); Serial.println(setpoint);
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

    char cmd[4] = {0};
    float val = 0.0f;

    if (serialLine.equalsIgnoreCase("P")) {
      if (!PLOTTER_MODE) printStatus();
    } else if (serialLine.equalsIgnoreCase("CAL")) {
      autoCalibrateMidpoint();
      iTerm = 0.0f;
      outPrev = 0.0f;
      if (!PLOTTER_MODE) Serial.println("Calibrated.");
    } else if (sscanf(serialLine.c_str(), "%3s %f", cmd, &val) == 2) {
      if (strcmp(cmd, "Kp") == 0 || strcmp(cmd, "KP") == 0) Kp = max(0.0f, val);
      else if (strcmp(cmd, "Ki") == 0 || strcmp(cmd, "KI") == 0) Ki = max(0.0f, val);
      else if (strcmp(cmd, "Kd") == 0 || strcmp(cmd, "KD") == 0) Kd = max(0.0f, val);
      else if (strcmp(cmd, "SP") == 0) setpoint = (int)constrain(val, 0.0f, 1023.0f);
      else if (strcmp(cmd, "MID") == 0) midpoint = (int)constrain(val, 0.0f, 1023.0f);
      if (!PLOTTER_MODE) printStatus();
    } else if (!PLOTTER_MODE) {
      Serial.println("Use: Kp v | Ki v | Kd v | SP v | MID v | CAL | P");
    }

    serialLine = "";
  }
}

void setup() {
  pinMode(PIN_LDR, INPUT);

  strip.begin();
  strip.clear();
  strip.show();

  Serial.begin(115200);
  autoCalibrateMidpoint();

  if (PLOTTER_MODE) {
    delay(1200);
    Serial.println("LDR,SP,Error,Brightness");
  } else {
    Serial.println("Commands: Kp v | Ki v | Kd v | SP v | MID v | CAL | P");
    printStatus();
    Serial.println("LDR,SP,Error,Brightness");
  }

  lastControlMs = millis();
}

void loop() {
  handleSerial();

  unsigned long now = millis();
  if ((now - lastControlMs) < SAMPLE_MS) return;
  lastControlMs += SAMPLE_MS;

  int raw = readLdrAveraged();
  pvFilt += SENSOR_ALPHA * ((float)raw - pvFilt);

  float error = plantSign * ((float)setpoint - pvFilt);
  if (fabs(error) < ERROR_DEADBAND) error = 0.0f;

  // Derivative on measurement avoids derivative kick from setpoint changes.
  float dMeas = (pvFilt - prevPv) / DT;
  prevPv = pvFilt;
  dMeasFilt += DERIV_ALPHA * (dMeas - dMeasFilt);

  float pTerm = Kp * error;
  float dTerm = -Kd * dMeasFilt;

  // Conditional integration anti-windup.
  float uNoI = pTerm + iTerm + dTerm;
  bool atHighAndWorsening = (uNoI > 255.0f && error > 0.0f);
  bool atLowAndWorsening = (uNoI < 0.0f && error < 0.0f);
  if (!(atHighAndWorsening || atLowAndWorsening)) {
    iTerm += Ki * error * DT;
  }
  iTerm = constrain(iTerm, -120.0f, 120.0f);

  float u = pTerm + iTerm + dTerm;
  u = constrain(u, 0.0f, 255.0f);

  // Slew limiting keeps visible output smooth without making response too slow.
  u = constrain(u, outPrev - MAX_STEP, outPrev + MAX_STEP);
  outPrev = u;

  int brightness = (int)(u + 0.5f);
  setWhiteBrightness(brightness);

  Serial.print((int)pvFilt);
  Serial.print(",");
  Serial.print(setpoint);
  Serial.print(",");
  Serial.print(error, 2);
  Serial.print(",");
  Serial.println(brightness);
}

