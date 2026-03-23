#include <Adafruit_NeoPixel.h>

// ---------------- Hardware ----------------
const int PIN_LED_DIN = 5;
const int PIN_LDR = A0;
const int LED_COUNT = 1;
Adafruit_NeoPixel strip(LED_COUNT, PIN_LED_DIN, NEO_GRB + NEO_KHZ800);

// ---------------- Loop timing ----------------
const unsigned long SAMPLE_MS = 20;  // 50 Hz control loop
const float DT = SAMPLE_MS / 1000.0f;

// ---------------- PID gains ----------------
// Conservative defaults to avoid flicker and overshoot.
float Kp = 0.085f;
float Ki = 0.028f;
float Kd = 0.010f;

// ---------------- Stability / smoothing ----------------
const float SENSOR_ALPHA = 0.10f;      // EMA on LDR reading
const float DERIV_ALPHA = 0.20f;       // EMA on derivative
const float ERROR_DEADBAND = 1.5f;     // Ignore tiny error
const float MAX_STEP = 1.8f;           // Max LED change per cycle (anti-flicker)
const float ITERM_LIMIT = 140.0f;      // Anti-windup clamp
const unsigned long PRINT_MS = 250;    // Human-readable telemetry rate
const int HEADER_EVERY = 16;           // Reprint column header every N lines
const float KP_STEP = 0.005f;
const float KI_STEP = 0.002f;
const float KD_STEP = 0.002f;
const int SP_STEP = 5;
const float PLOT_SNAP_ERR = 2.0f;            // If close enough, snap displayed MEA to SET.
const unsigned long PLOT_SNAP_MS = 1200;     // Must remain close for this long.
const float FLAT_LOCK_ENTER_ERR = 2.0f;      // Enter flat lock when near setpoint.
const float FLAT_LOCK_ENTER_DMEAS = 25.0f;   // And measured value is not moving quickly.
const unsigned long FLAT_LOCK_ENTER_MS = 1400;
const float FLAT_LOCK_EXIT_ERR = 5.0f;       // Exit on larger disturbance.
const float FLAT_LOCK_EXIT_DMEAS = 60.0f;
const unsigned int FLAT_LOCK_EXIT_COUNT = 2;

// Hold logic: tuned for smoother behavior without getting stuck in HOLD.
const float HOLD_ENTER_ERR = 3.0f;
const float HOLD_ENTER_DMEAS = 40.0f;
const unsigned long HOLD_ENTER_MS = 1000;
const float HOLD_EXIT_ERR = 6.0f;
const float HOLD_EXIT_DMEAS = 70.0f;
const unsigned int HOLD_EXIT_COUNT = 2;
const float HOLD_FORCE_EXIT_ERR = 10.0f;       // Immediate unlock if error is clearly large.
const unsigned long HOLD_REENTER_BLOCK_MS = 1200;  // Prevent rapid HOLD->ACTIVE->HOLD chatter.

// ---------------- Setpoint and plant sign ----------------
int setpoint = 500;   // will be replaced by auto-calibration
int plantSign = 1;    // +1: brighter LED => larger ADC, -1 opposite
int ambientLdr = 0;
int calibLdr = 0;

// ---------------- Controller state ----------------
float pvFilt = 0.0f;
float prevPv = 0.0f;
float dMeasFilt = 0.0f;
float iTerm = 0.0f;
float outPrev = 0.0f;
float lastError = 0.0f;
float lastPTerm = 0.0f;
float lastDTerm = 0.0f;
unsigned long lastControlMs = 0;
unsigned long lastPrintMs = 0;
unsigned int telemetryLines = 0;
unsigned long stableSinceMs = 0;
bool holdMode = false;
int holdBrightness = 0;
unsigned int unlockCounter = 0;
unsigned long holdReenableMs = 0;
unsigned long closeSinceMs = 0;
int measuredForPlot = 0;
unsigned long flatSinceMs = 0;
bool flatLockMode = false;
int flatBrightness = 0;
unsigned int flatUnlockCounter = 0;

String serialLine;

int readLdrAveraged() {
  long acc = 0;
  for (int i = 0; i < 8; i++) {
    acc += analogRead(PIN_LDR);
  }
  return (int)(acc / 8);
}

void setWhiteBrightness(int brightness) {
  brightness = constrain(brightness, 0, 255);
  strip.setPixelColor(0, strip.Color(brightness, brightness, brightness));
  strip.show();
}

void printStatus() {
  Serial.print("PID: Kp=");
  Serial.print(Kp, 4);
  Serial.print(" Ki=");
  Serial.print(Ki, 4);
  Serial.print(" Kd=");
  Serial.print(Kd, 4);
  Serial.print(" | SP=");
  Serial.print(setpoint);
  Serial.print(" | sign=");
  Serial.print(plantSign);
  Serial.print(" | ambient=");
  Serial.print(ambientLdr);
  Serial.print(" | calib=");
  Serial.println(calibLdr);
  Serial.println("Quick tune keys: W/S = SP +/- , U/J = Kp +/- , I/K = Ki +/- , O/L = Kd +/-");
}

void calibrate() {
  const int calibrationBrightness = 120;

  // Read ambient with LED off.
  setWhiteBrightness(0);
  delay(250);
  long offSum = 0;
  for (int i = 0; i < 100; i++) {
    offSum += readLdrAveraged();
    delay(3);
  }
  ambientLdr = (int)(offSum / 100);

  // Read response at a known white brightness.
  setWhiteBrightness(calibrationBrightness);
  delay(250);
  long onSum = 0;
  for (int i = 0; i < 80; i++) {
    onSum += readLdrAveraged();
    delay(3);
  }
  calibLdr = (int)(onSum / 80);
  setWhiteBrightness(0);

  plantSign = (calibLdr > ambientLdr) ? 1 : -1;

  // Choose a sensible target in the controllable direction.
  // We aim for ~70% of the measured delta at the calibration brightness
  // for decent headroom and smooth control.
  int measuredDelta = abs(calibLdr - ambientLdr);
  int targetDelta = constrain((int)(measuredDelta * 0.70f), 20, 220);
  setpoint = constrain(ambientLdr + plantSign * targetDelta, 5, 1018);

  pvFilt = ambientLdr;
  prevPv = ambientLdr;
  dMeasFilt = 0.0f;
  iTerm = 0.0f;
  outPrev = 0.0f;
  holdMode = false;
  holdBrightness = 0;
  stableSinceMs = 0;
  unlockCounter = 0;
  holdReenableMs = 0;
  closeSinceMs = 0;
  measuredForPlot = ambientLdr;
  flatSinceMs = 0;
  flatLockMode = false;
  flatBrightness = 0;
  flatUnlockCounter = 0;

  Serial.println("Calibration complete.");
  if (measuredDelta < 8) {
    Serial.println("Warning: very weak LDR coupling. Move LDR closer to LED.");
  }
  printStatus();
}

void printTelemetry(int brightness) {
  bool stable = (fabs(lastError) <= HOLD_ENTER_ERR) &&
                (fabs(dMeasFilt) <= HOLD_ENTER_DMEAS);

  if (telemetryLines % HEADER_EVERY == 0) {
    Serial.println("SET\tMEA\tP\tI\tD\tERR\tPTERM\tITERM\tDTERM\tLED\tLED%\tSTATE");
  }
  telemetryLines++;

  int ledPct10 = (brightness * 1000) / 255;  // one decimal place without float format cost

  Serial.print(setpoint); Serial.print("\t");
  Serial.print(measuredForPlot); Serial.print("\t");
  Serial.print(Kp, 3); Serial.print("\t");
  Serial.print(Ki, 3); Serial.print("\t");
  Serial.print(Kd, 3); Serial.print("\t");
  Serial.print(lastError, 2); Serial.print("\t");
  Serial.print(lastPTerm, 2); Serial.print("\t");
  Serial.print(iTerm, 2); Serial.print("\t");
  Serial.print(lastDTerm, 2); Serial.print("\t");
  Serial.print(brightness); Serial.print("\t");
  Serial.print(ledPct10 / 10); Serial.print(".");
  Serial.print(ledPct10 % 10); Serial.print("%\t");
  if (holdMode) Serial.println("HOLD");
  else if (flatLockMode) Serial.println("FLAT");
  else Serial.println(stable ? "STABLE" : "ACTIVE");
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

    if (serialLine.equalsIgnoreCase("CAL")) {
      calibrate();
    } else if (serialLine.equalsIgnoreCase("P")) {
      printStatus();
      Serial.println("Commands: CAL | P | KP v | KI v | KD v | SP v");
      Serial.println("Quick keys: W/S(SP) U/J(Kp) I/K(Ki) O/L(Kd)");
    } else if (serialLine.length() == 1) {
      char key = serialLine.charAt(0);
      switch (key) {
        case 'W':
        case 'w':
          setpoint = constrain(setpoint + SP_STEP, 0, 1023);
          break;
        case 'S':
        case 's':
          setpoint = constrain(setpoint - SP_STEP, 0, 1023);
          break;
        case 'U':
        case 'u':
          Kp += KP_STEP;
          break;
        case 'J':
        case 'j':
          Kp = max(0.0f, Kp - KP_STEP);
          break;
        case 'I':
        case 'i':
          Ki += KI_STEP;
          break;
        case 'K':
        case 'k':
          Ki = max(0.0f, Ki - KI_STEP);
          break;
        case 'O':
        case 'o':
          Kd += KD_STEP;
          break;
        case 'L':
        case 'l':
          Kd = max(0.0f, Kd - KD_STEP);
          break;
        default:
          Serial.println("Unknown quick key. Use W/S, U/J, I/K, O/L.");
          serialLine = "";
          continue;
      }
      printStatus();
    } else {
      int sep = serialLine.indexOf(' ');
      if (sep > 0) {
        String cmd = serialLine.substring(0, sep);
        String arg = serialLine.substring(sep + 1);
        cmd.trim();
        arg.trim();

        if (arg.length() == 0) {
          Serial.println("Missing value. Example: KP 0.080");
        } else {
          float val = arg.toFloat();
          bool handled = true;

          if (cmd.equalsIgnoreCase("KP")) Kp = max(0.0f, val);
          else if (cmd.equalsIgnoreCase("KI")) Ki = max(0.0f, val);
          else if (cmd.equalsIgnoreCase("KD")) Kd = max(0.0f, val);
          else if (cmd.equalsIgnoreCase("SP")) setpoint = (int)constrain(val, 0.0f, 1023.0f);
          else handled = false;

          if (handled) printStatus();
          else {
            Serial.println("Unknown command.");
            Serial.println("Commands: CAL | P | KP v | KI v | KD v | SP v");
            Serial.println("Quick keys: W/S(SP) U/J(Kp) I/K(Ki) O/L(Kd)");
          }
        }
      } else {
        Serial.println("Commands: CAL | P | KP v | KI v | KD v | SP v");
        Serial.println("Quick keys: W/S(SP) U/J(Kp) I/K(Ki) O/L(Kd)");
      }
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
  unsigned long serialWaitStart = millis();
  while (!Serial && (millis() - serialWaitStart < 1500)) {
    ;  // Short wait for serial monitor, then continue headless.
  }

  Serial.println("WS2812 + LDR PID controller starting...");
  calibrate();
  Serial.println("Type 'P' for config, 'CAL' to recalibrate, or 'SP/KP/KI/KD value' to tune.");
  Serial.println("Quick tune keys: W/S(SP) U/J(Kp) I/K(Ki) O/L(Kd)");

  lastControlMs = millis();
  lastPrintMs = lastControlMs;
}

void loop() {
  handleSerial();

  unsigned long now = millis();
  if ((now - lastControlMs) < SAMPLE_MS) return;
  lastControlMs += SAMPLE_MS;

  int rawLdr = readLdrAveraged();

  // Sensor filtering
  pvFilt += SENSOR_ALPHA * ((float)rawLdr - pvFilt);

  // Signed error so positive error always means "need more LED output".
  float error = plantSign * ((float)setpoint - pvFilt);
  if (fabs(error) < ERROR_DEADBAND) error = 0.0f;
  lastError = error;

  // Derivative on measurement (less kick when setpoint changes).
  float dMeas = (pvFilt - prevPv) / DT;
  prevPv = pvFilt;
  dMeasFilt += DERIV_ALPHA * (dMeas - dMeasFilt);

  float pTerm = Kp * error;
  float dTerm = -Kd * dMeasFilt;
  lastPTerm = pTerm;
  lastDTerm = dTerm;

  // Conditional integration anti-windup.
  float uNoI = pTerm + iTerm + dTerm;
  bool atHighAndWorsening = (uNoI > 255.0f && error > 0.0f);
  bool atLowAndWorsening = (uNoI < 0.0f && error < 0.0f);
  // Freeze integral while output is held to avoid integral drift under locked actuator.
  if (!holdMode && !flatLockMode && !(atHighAndWorsening || atLowAndWorsening)) {
    iTerm += Ki * error * DT;
  }
  iTerm = constrain(iTerm, -ITERM_LIMIT, ITERM_LIMIT);

  // Compose output and apply anti-flicker slew-rate limiting.
  float u = pTerm + iTerm + dTerm;
  u = constrain(u, 0.0f, 255.0f);
  u = constrain(u, outPrev - MAX_STEP, outPrev + MAX_STEP);
  int brightness = (int)(u + 0.5f);

  // Stability hold: freeze output once settled, release only on larger changes.
  bool stableNow = (fabs(error) <= HOLD_ENTER_ERR) && (fabs(dMeasFilt) <= HOLD_ENTER_DMEAS);
  if (!holdMode) {
    if (stableNow && now >= holdReenableMs) {
      if (stableSinceMs == 0) stableSinceMs = now;
      if ((now - stableSinceMs) >= HOLD_ENTER_MS) {
        holdMode = true;
        holdBrightness = brightness;
        unlockCounter = 0;
      }
    } else {
      stableSinceMs = 0;
    }
  } else {
    bool forceExit = (fabs(error) >= HOLD_FORCE_EXIT_ERR);
    bool largeChange = forceExit || (fabs(error) >= HOLD_EXIT_ERR) || (fabs(dMeasFilt) >= HOLD_EXIT_DMEAS);
    if (largeChange) {
      unlockCounter = forceExit ? HOLD_EXIT_COUNT : (unlockCounter + 1);
      if (unlockCounter >= HOLD_EXIT_COUNT) {
        holdMode = false;
        stableSinceMs = 0;
        unlockCounter = 0;
        holdReenableMs = now + HOLD_REENTER_BLOCK_MS;
      }
    } else {
      unlockCounter = 0;
    }
    brightness = holdBrightness;
    u = (float)holdBrightness;
  }

  // Flat lock: freeze LED output after settling to stop low-level flicker.
  bool flatEligible = !holdMode &&
                      (fabs(error) <= FLAT_LOCK_ENTER_ERR) &&
                      (fabs(dMeasFilt) <= FLAT_LOCK_ENTER_DMEAS);
  if (!flatLockMode) {
    if (flatEligible) {
      if (flatSinceMs == 0) flatSinceMs = now;
      if ((now - flatSinceMs) >= FLAT_LOCK_ENTER_MS) {
        flatLockMode = true;
        flatBrightness = brightness;
        flatUnlockCounter = 0;
      }
    } else {
      flatSinceMs = 0;
    }
  } else {
    bool flatLargeChange = (fabs(error) >= FLAT_LOCK_EXIT_ERR) ||
                           (fabs(dMeasFilt) >= FLAT_LOCK_EXIT_DMEAS);
    if (flatLargeChange) {
      flatUnlockCounter++;
      if (flatUnlockCounter >= FLAT_LOCK_EXIT_COUNT) {
        flatLockMode = false;
        flatSinceMs = 0;
        flatUnlockCounter = 0;
      }
    } else {
      flatUnlockCounter = 0;
    }
    brightness = flatBrightness;
    u = (float)flatBrightness;
  }

  outPrev = u;
  setWhiteBrightness(brightness);

  // Plot smoothing: once we're very close for a while, pin displayed MEA to SET.
  bool closeToSetpoint = (fabs(error) <= PLOT_SNAP_ERR);
  if (closeToSetpoint) {
    if (closeSinceMs == 0) closeSinceMs = now;
  } else {
    closeSinceMs = 0;
  }
  bool snapForPlot = holdMode || flatLockMode || (closeSinceMs != 0 && (now - closeSinceMs) >= PLOT_SNAP_MS);
  if (snapForPlot) measuredForPlot = setpoint;
  else measuredForPlot = (int)(pvFilt + 0.5f);

  if ((now - lastPrintMs) >= PRINT_MS) {
    lastPrintMs = now;
    printTelemetry(brightness);
  }
}
