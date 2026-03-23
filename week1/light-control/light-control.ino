#include <Adafruit_NeoPixel.h>

// Hardware
const int PIN_LED_DIN = 5;
const int PIN_LDR = A0;
const int LED_COUNT = 1;
Adafruit_NeoPixel strip(LED_COUNT, PIN_LED_DIN, NEO_GRB + NEO_KHZ800);

// Control loop
const unsigned long SAMPLE_MS = 20; // 50 Hz
const float DT = SAMPLE_MS / 1000.0f;

// PID gains
float Kp = 0.07f;
float Ki = 0.01f;
float Kd = 0.004f;

// Signal conditioning / output shaping
const float SENSOR_ALPHA = 0.12f;
const float DERIV_ALPHA = 0.25f;
const float ERROR_DEADBAND = 2.0f;
const float MAX_STEP = 2.0f;

// Setpoint setup
int setpoint = 550;
int plantSign = 1; // +1 if LED increase raises LDR reading, -1 otherwise

// State
float pvFilt = 0.0f;
float prevPv = 0.0f;
float dMeasFilt = 0.0f;
float iTerm = 0.0f;
float outPrev = 0.0f;
unsigned long lastControlMs = 0;

String serialLine;

int readLdrAveraged() {
  int acc = 0;
  for (int i = 0; i < 8; i++) acc += analogRead(PIN_LDR);
  return acc / 8;
}

void setWhiteBrightness(int brightness) {
  brightness = constrain(brightness, 0, 255);
  strip.setPixelColor(0, strip.Color(brightness, brightness, brightness));
  strip.show();
}

void calibrate() {
  // Measure ambient with LED off.
  setWhiteBrightness(0);
  delay(200);
  long offSum = 0;
  for (int i = 0; i < 80; i++) {
    offSum += readLdrAveraged();
    delay(4);
  }
  int ambient = (int)(offSum / 80);

  // Detect whether LED increases or decreases LDR reading.
  setWhiteBrightness(80);
  delay(180);
  long onSum = 0;
  for (int i = 0; i < 40; i++) {
    onSum += readLdrAveraged();
    delay(4);
  }
  int lit = (int)(onSum / 40);
  setWhiteBrightness(0);

  plantSign = (lit > ambient) ? 1 : -1;

  // Place setpoint in controllable direction from ambient.
  const int offset = 120;
  setpoint = constrain(ambient + plantSign * offset, 10, 1010);

  pvFilt = ambient;
  prevPv = ambient;
  iTerm = 0.0f;
  outPrev = 0.0f;
}

void printStatus() {
  Serial.print("Kp="); Serial.print(Kp, 4);
  Serial.print(" Ki="); Serial.print(Ki, 4);
  Serial.print(" Kd="); Serial.print(Kd, 4);
  Serial.print(" SP="); Serial.print(setpoint);
  Serial.print(" S="); Serial.println(plantSign);
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

    if (serialLine.equalsIgnoreCase("CAL")) {
      calibrate();
      Serial.println("Calibrated");
      printStatus();
    } else if (serialLine.equalsIgnoreCase("P")) {
      printStatus();
    } else if (sscanf(serialLine.c_str(), "%3s %f", cmd, &val) == 2) {
      if (strcmp(cmd, "Kp") == 0 || strcmp(cmd, "KP") == 0) Kp = max(0.0f, val);
      else if (strcmp(cmd, "Ki") == 0 || strcmp(cmd, "KI") == 0) Ki = max(0.0f, val);
      else if (strcmp(cmd, "Kd") == 0 || strcmp(cmd, "KD") == 0) Kd = max(0.0f, val);
      else if (strcmp(cmd, "SP") == 0) setpoint = (int)constrain(val, 0.0f, 1023.0f);
      printStatus();
    } else {
      Serial.println("Commands: CAL | P | Kp v | Ki v | Kd v | SP v");
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
  calibrate();

  delay(1000);
  Serial.println("time_ms,process_value,setpoint,error,brightness");

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

  float dMeas = (pvFilt - prevPv) / DT;
  prevPv = pvFilt;
  dMeasFilt += DERIV_ALPHA * (dMeas - dMeasFilt);

  float pTerm = Kp * error;
  float dTerm = -Kd * dMeasFilt;

  float uNoI = pTerm + iTerm + dTerm;
  bool atHighAndWorsening = (uNoI > 255.0f && error > 0.0f);
  bool atLowAndWorsening = (uNoI < 0.0f && error < 0.0f);
  if (!(atHighAndWorsening || atLowAndWorsening)) {
    iTerm += Ki * error * DT;
  }
  iTerm = constrain(iTerm, -120.0f, 120.0f);

  float u = pTerm + iTerm + dTerm;
  u = constrain(u, 0.0f, 255.0f);
  u = constrain(u, outPrev - MAX_STEP, outPrev + MAX_STEP);
  outPrev = u;

  int brightness = (int)(u + 0.5f);
  setWhiteBrightness(brightness);

  Serial.print(now);
  Serial.print(",");
  Serial.print((int)pvFilt);
  Serial.print(",");
  Serial.print(setpoint);
  Serial.print(",");
  Serial.print(error, 3);
  Serial.print(",");
  Serial.println(brightness);
}
