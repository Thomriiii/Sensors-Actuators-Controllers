/*
  Rotary inverted pendulum controller for:
    - Arduino Uno Rev3
    - Arduino Motor Shield Rev3
    - 42BYGH162-A-21DH bipolar stepper motor
    - E38S6G5-600B-G24N incremental encoder

  Working assumptions used in this sketch:
    1. The encoder is mounted on the pendulum pivot, so it measures pendulum
       angle directly.
    2. The arm angle is estimated from commanded stepper steps because the rig
       has only one encoder in its current form.
    3. Because the Motor Shield already uses D3 for motor PWM, the encoder is
       decoded from channel A interrupts only. This yields 1200 edge counts per
       revolution from a 600 pulse encoder, rather than full x4 decoding.

  Control architecture:
    - Energy-based swing-up when the pendulum is away from upright.
    - State-feedback balancing once the pendulum enters a capture region.
    - Step-rate and acceleration limiting to reduce skipped steps.
    - The start/hanging position is treated as 0 degrees. The controller
      targets the upright position at 180 degrees.

  Important practical limitation:
    If the stepper skips steps, the estimated arm angle drifts away from the
    true arm angle. That weakens both swing-up and balancing performance.
*/

#include <Arduino.h>

// -------------------- Motor Shield Rev3 pin map --------------------
const uint8_t PIN_MOTOR_A_DIR = 12;
const uint8_t PIN_MOTOR_A_PWM = 3;
const uint8_t PIN_MOTOR_A_BRAKE = 9;

const uint8_t PIN_MOTOR_B_DIR = 13;
const uint8_t PIN_MOTOR_B_PWM = 11;
const uint8_t PIN_MOTOR_B_BRAKE = 8;

// -------------------- Encoder and safety IO --------------------
const uint8_t PIN_ENCODER_A = 2;  // External interrupt pin
const uint8_t PIN_ENCODER_B = A4;  // Direction state pin
const uint8_t PIN_ESTOP = 6;      // Active-low emergency stop

// -------------------- Timing --------------------
const unsigned long CONTROL_SAMPLE_US = 2500UL;  // 400 Hz
unsigned long lastControlSampleUs = 0;
unsigned long lastStepMicros = 0;

// -------------------- Encoder and geometry --------------------
const long ENCODER_PULSES_PER_REV = 600L;
const long ENCODER_EDGES_PER_REV = 2L * ENCODER_PULSES_PER_REV; // Channel A CHANGE decoding
const long ENCODER_HALF_TURN_EDGES = ENCODER_EDGES_PER_REV / 2L;
const float TWO_PI_F = 6.28318530718f;
const float ENCODER_RAD_PER_EDGE = TWO_PI_F / (float)ENCODER_EDGES_PER_REV;
const float PENDULUM_TARGET_RAD = PI;     // Upright target is 180 deg from start.
const float PENDULUM_DISPLAY_OFFSET_RAD = PI; // Encoder zero is physically upright on this rig.

const long STEPPER_FULL_STEPS_PER_REV = 200L;  // 1.8 degree stepper
const float ARM_RAD_PER_STEP = TWO_PI_F / (float)STEPPER_FULL_STEPS_PER_REV;
const long ARM_LIMIT_STEPS = STEPPER_FULL_STEPS_PER_REV / 4L; // +/-90 deg from reset centre.
const float ARM_LIMIT_RAD = PI / 2.0f;

// -------------------- Simple plant estimates used by control law --------------------
const float GRAVITY = 9.81f;
const float PENDULUM_MASS_KG = 0.03015f;
const float PENDULUM_COM_M = 0.08366f;
const float PENDULUM_INERTIA_KGM2 = 0.00014256f;
const float PENDULUM_EFFECTIVE_INERTIA = PENDULUM_INERTIA_KGM2 +
                                         PENDULUM_MASS_KG * PENDULUM_COM_M * PENDULUM_COM_M;
const float SWING_UP_DESIRED_ENERGY = 2.0f * PENDULUM_MASS_KG * GRAVITY * PENDULUM_COM_M;

// -------------------- Controller gains --------------------
const float BALANCE_K_THETA = 0.55f;
const float BALANCE_K_ALPHA = 12.0f;
const float BALANCE_K_THETA_DOT = 0.20f;
const float BALANCE_K_ALPHA_DOT = 1.55f;

const float CONTROL_COMMAND_POLARITY = -1.0f;

const float SWING_ENERGY_GAIN = 3.25f;
const float SWING_ARM_CENTER_GAIN = 0.22f;
const float SWING_ARM_RATE_GAIN = 0.06f;
const float SWING_MIN_COMMAND = 0.35f;
const float MOTOR_TEST_COMMAND = 0.35f;
const unsigned long MOTOR_TEST_DURATION_MS = 1200UL;

const float BALANCE_ENTRY_ANGLE_RAD = 12.0f * DEG_TO_RAD;
const float BALANCE_EXIT_ANGLE_RAD = 18.0f * DEG_TO_RAD;
const float BALANCE_ENTRY_RATE_RAD_S = 2.5f;

// -------------------- Stepper command shaping --------------------
const float MAX_STEP_RATE = 900.0f;       // full steps/s
const float MAX_STEP_ACCEL = 9000.0f;     // full steps/s^2
const uint8_t MAX_COIL_PWM = 220;
const uint8_t HOLD_COIL_PWM = 130;
const uint8_t BALANCE_HOLD_PWM = 175;

// -------------------- State estimation --------------------
const float RATE_FILTER_ALPHA = 0.25f;
volatile long pendulumEncoderEdges = 0;

long encoderStartEdges = 0;
long commandedArmSteps = 0;

float pendulumAngleRad = -PI;             // Error from 180 deg upright, wrapped to [-pi, pi]
float pendulumPositionRad = 0.0f;         // Start/hanging position is 0 deg
float pendulumContinuousRad = 0.0f;       // Unwrapped position for rate estimation
float pendulumRateRadS = 0.0f;

float armAngleRad = 0.0f;                 // Estimated from commanded steps
float armRateRadS = 0.0f;

float prevPendulumContinuousRad = 0.0f;
float prevArmContinuousRad = 0.0f;

float demandedStepRate = 0.0f;
float appliedStepRate = 0.0f;
uint8_t appliedCoilPwm = 0;
bool motorHoldingEnabled = false;
uint8_t currentStepIndex = 0;
int8_t motorTestDirection = 0;
unsigned long motorTestUntilMs = 0;

// -------------------- Control mode --------------------
enum ControlMode {
  MODE_IDLE = 0,
  MODE_SWING_UP = 1,
  MODE_BALANCE = 2
};

ControlMode controlMode = MODE_IDLE;
bool controllerEnabled = false;

// -------------------- Function declarations --------------------
void encoderAChangeISR();
void updateStateEstimate(float dt);
void updateControlMode();
float computeSwingUpCommand();
float computeBalanceCommand();
void applyNormalisedCommand(float command, float dt);
float applyArmTravelLimit(float command);
void serviceStepper();
void singleStep(int direction);
void applyStepState(uint8_t stepIndex, uint8_t pwmValue);
void setBridge(uint8_t dirPin, uint8_t pwmPin, uint8_t brakePin, int sign, uint8_t pwmValue);
void releaseStepper();
void stopController();
void resetArmEstimate();
void startMotorTest(int direction);
void printHelp();
void printStatus();
void handleSerial();
long readPendulumEdges();
float wrapToTwoPi(float angleRad);
float wrapToPi(float angleRad);
float constrainFloat(float value, float low, float high);
float signNoZero(float value);

void setup() {
  pinMode(PIN_MOTOR_A_DIR, OUTPUT);
  pinMode(PIN_MOTOR_A_PWM, OUTPUT);
  pinMode(PIN_MOTOR_A_BRAKE, OUTPUT);
  pinMode(PIN_MOTOR_B_DIR, OUTPUT);
  pinMode(PIN_MOTOR_B_PWM, OUTPUT);
  pinMode(PIN_MOTOR_B_BRAKE, OUTPUT);

  pinMode(PIN_ENCODER_A, INPUT_PULLUP);
  pinMode(PIN_ENCODER_B, INPUT_PULLUP);
  pinMode(PIN_ESTOP, INPUT_PULLUP);

  releaseStepper();

  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_A), encoderAChangeISR, CHANGE);
  encoderStartEdges = readPendulumEdges();

  Serial.begin(115200);
  while (!Serial) {
    ;
  }

  Serial.println(F("Rotary inverted pendulum controller"));
  Serial.println(F("Use serial commands to calibrate before enabling control."));
  printHelp();

  lastControlSampleUs = micros();
  lastStepMicros = lastControlSampleUs;
}

void loop() {
  serviceStepper();
  handleSerial();

  unsigned long now = micros();
  if (now - lastControlSampleUs < CONTROL_SAMPLE_US) {
    return;
  }

  float dt = (now - lastControlSampleUs) * 1.0e-6f;
  lastControlSampleUs = now;

  if (digitalRead(PIN_ESTOP) == LOW) {
    stopController();
    Serial.println(F("E-stop active. Controller disabled."));
    delay(20);
    return;
  }

  updateStateEstimate(dt);

  if (motorTestDirection != 0) {
    if (millis() < motorTestUntilMs) {
      controlMode = MODE_IDLE;
      controllerEnabled = false;
      applyNormalisedCommand((float)motorTestDirection * MOTOR_TEST_COMMAND, dt);
    } else {
      motorTestDirection = 0;
      stopController();
      Serial.println(F("Motor test finished."));
    }
  } else if (!controllerEnabled) {
    controlMode = MODE_IDLE;
    demandedStepRate = 0.0f;
    applyNormalisedCommand(0.0f, dt);
  } else {
    updateControlMode();

    float command = 0.0f;
    if (controlMode == MODE_BALANCE) {
      command = computeBalanceCommand();
    } else {
      command = computeSwingUpCommand();
    }

    command *= CONTROL_COMMAND_POLARITY;
    applyNormalisedCommand(command, dt);
  }

  static uint8_t printDivider = 0;
  if (++printDivider >= 40) {
    printDivider = 0;

    Serial.print(F("mode="));
    if (controlMode == MODE_IDLE) {
      Serial.print(F("idle"));
    } else if (controlMode == MODE_SWING_UP) {
      Serial.print(F("swing"));
    } else {
      Serial.print(F("balance"));
    }

    Serial.print(F(", pendulum_deg="));
    Serial.print(pendulumPositionRad * RAD_TO_DEG, 2);
    Serial.print(F(", target_error_deg="));
    Serial.print(pendulumAngleRad * RAD_TO_DEG, 2);
    Serial.print(F(", alpha_dot="));
    Serial.print(pendulumRateRadS, 3);
    Serial.print(F(", theta_deg_est="));
    Serial.print(armAngleRad * RAD_TO_DEG, 2);
    Serial.print(F(", arm_limit_deg=+/-"));
    Serial.print(ARM_LIMIT_RAD * RAD_TO_DEG, 0);
    Serial.print(F(", step_rate="));
    Serial.print(appliedStepRate, 1);
    Serial.print(F(", pwm="));
    Serial.println(appliedCoilPwm);
  }
}

void encoderAChangeISR() {
  bool aState = digitalRead(PIN_ENCODER_A);
  bool bState = digitalRead(PIN_ENCODER_B);

  if (aState == bState) {
    pendulumEncoderEdges++;
  } else {
    pendulumEncoderEdges--;
  }
}

void updateStateEstimate(float dt) {
  long edges = readPendulumEdges();

  pendulumContinuousRad = (float)(edges - encoderStartEdges) * ENCODER_RAD_PER_EDGE;
  float rawPendulumRate = (pendulumContinuousRad - prevPendulumContinuousRad) / dt;
  pendulumRateRadS += RATE_FILTER_ALPHA * (rawPendulumRate - pendulumRateRadS);
  prevPendulumContinuousRad = pendulumContinuousRad;
  pendulumPositionRad = wrapToTwoPi(pendulumContinuousRad + PENDULUM_DISPLAY_OFFSET_RAD);
  pendulumAngleRad = wrapToPi(pendulumPositionRad - PENDULUM_TARGET_RAD);

  float armContinuousRad = (float)commandedArmSteps * ARM_RAD_PER_STEP;
  float rawArmRate = (armContinuousRad - prevArmContinuousRad) / dt;
  armRateRadS += RATE_FILTER_ALPHA * (rawArmRate - armRateRadS);
  prevArmContinuousRad = armContinuousRad;
  armAngleRad = armContinuousRad;
}

void updateControlMode() {
  bool insideBalanceWindow = fabs(pendulumAngleRad) < BALANCE_ENTRY_ANGLE_RAD &&
                             fabs(pendulumRateRadS) < BALANCE_ENTRY_RATE_RAD_S;

  if (controlMode != MODE_BALANCE && insideBalanceWindow) {
    controlMode = MODE_BALANCE;
  } else if (controlMode == MODE_BALANCE && fabs(pendulumAngleRad) > BALANCE_EXIT_ANGLE_RAD) {
    controlMode = MODE_SWING_UP;
  } else if (controlMode == MODE_IDLE) {
    controlMode = MODE_SWING_UP;
  }
}

float computeSwingUpCommand() {
  float energy = 0.5f * PENDULUM_EFFECTIVE_INERTIA * pendulumRateRadS * pendulumRateRadS +
                 PENDULUM_MASS_KG * GRAVITY * PENDULUM_COM_M * (1.0f + cos(pendulumAngleRad));

  float phase = pendulumRateRadS * cos(pendulumAngleRad);
  if (fabs(phase) < 0.05f) {
    phase = sin(pendulumAngleRad);
  }

  float command = SWING_ENERGY_GAIN * (SWING_UP_DESIRED_ENERGY - energy) * signNoZero(phase)
                - SWING_ARM_CENTER_GAIN * armAngleRad
                - SWING_ARM_RATE_GAIN * armRateRadS;

  command = constrainFloat(command, -1.0f, 1.0f);
  if (fabs(command) < SWING_MIN_COMMAND) {
    command = SWING_MIN_COMMAND * signNoZero(command + phase);
  }

  return command;
}

float computeBalanceCommand() {
  float command = -(BALANCE_K_THETA * armAngleRad +
                    BALANCE_K_ALPHA * pendulumAngleRad +
                    BALANCE_K_THETA_DOT * armRateRadS +
                    BALANCE_K_ALPHA_DOT * pendulumRateRadS);

  return constrainFloat(command, -1.0f, 1.0f);
}

void applyNormalisedCommand(float command, float dt) {
  command = applyArmTravelLimit(command);
  demandedStepRate = command * MAX_STEP_RATE;

  float maxRateChange = MAX_STEP_ACCEL * dt;
  float rateError = demandedStepRate - appliedStepRate;
  rateError = constrainFloat(rateError, -maxRateChange, maxRateChange);
  appliedStepRate += rateError;
  if ((commandedArmSteps >= ARM_LIMIT_STEPS && appliedStepRate > 0.0f) ||
      (commandedArmSteps <= -ARM_LIMIT_STEPS && appliedStepRate < 0.0f)) {
    appliedStepRate = 0.0f;
  }

  float commandMagnitude = fabs(command);
  appliedCoilPwm = HOLD_COIL_PWM + (uint8_t)((MAX_COIL_PWM - HOLD_COIL_PWM) * commandMagnitude);
  if (controlMode == MODE_BALANCE && appliedCoilPwm < BALANCE_HOLD_PWM) {
    appliedCoilPwm = BALANCE_HOLD_PWM;
  }

  motorHoldingEnabled = controllerEnabled;
  if (motorTestDirection != 0) {
    motorHoldingEnabled = true;
  }

  if (!controllerEnabled && motorTestDirection == 0) {
    appliedStepRate = 0.0f;
    appliedCoilPwm = 0;
    motorHoldingEnabled = false;
  }
}

float applyArmTravelLimit(float command) {
  if ((commandedArmSteps >= ARM_LIMIT_STEPS && command > 0.0f) ||
      (commandedArmSteps <= -ARM_LIMIT_STEPS && command < 0.0f)) {
    return 0.0f;
  }

  return command;
}

void serviceStepper() {
  if (!motorHoldingEnabled) {
    releaseStepper();
    return;
  }

  applyStepState(currentStepIndex, appliedCoilPwm);

  if (fabs(appliedStepRate) < 0.5f) {
    return;
  }

  unsigned long now = micros();
  unsigned long stepIntervalUs = (unsigned long)(1000000.0f / fabs(appliedStepRate));

  if (stepIntervalUs < 800UL) {
    stepIntervalUs = 800UL;
  }

  if (now - lastStepMicros >= stepIntervalUs) {
    int direction = (appliedStepRate >= 0.0f) ? 1 : -1;
    singleStep(direction);
    lastStepMicros = now;
  }
}

void singleStep(int direction) {
  long nextArmSteps = commandedArmSteps + ((direction > 0) ? 1L : -1L);
  if (nextArmSteps > ARM_LIMIT_STEPS || nextArmSteps < -ARM_LIMIT_STEPS) {
    appliedStepRate = 0.0f;
    demandedStepRate = 0.0f;
    return;
  }

  if (direction > 0) {
    currentStepIndex = (currentStepIndex + 1U) & 0x03U;
    commandedArmSteps++;
  } else {
    currentStepIndex = (currentStepIndex + 3U) & 0x03U;
    commandedArmSteps--;
  }

  applyStepState(currentStepIndex, appliedCoilPwm);
}

void applyStepState(uint8_t stepIndex, uint8_t pwmValue) {
  switch (stepIndex & 0x03U) {
    case 0:
      setBridge(PIN_MOTOR_A_DIR, PIN_MOTOR_A_PWM, PIN_MOTOR_A_BRAKE, +1, pwmValue);
      setBridge(PIN_MOTOR_B_DIR, PIN_MOTOR_B_PWM, PIN_MOTOR_B_BRAKE, +1, pwmValue);
      break;
    case 1:
      setBridge(PIN_MOTOR_A_DIR, PIN_MOTOR_A_PWM, PIN_MOTOR_A_BRAKE, -1, pwmValue);
      setBridge(PIN_MOTOR_B_DIR, PIN_MOTOR_B_PWM, PIN_MOTOR_B_BRAKE, +1, pwmValue);
      break;
    case 2:
      setBridge(PIN_MOTOR_A_DIR, PIN_MOTOR_A_PWM, PIN_MOTOR_A_BRAKE, -1, pwmValue);
      setBridge(PIN_MOTOR_B_DIR, PIN_MOTOR_B_PWM, PIN_MOTOR_B_BRAKE, -1, pwmValue);
      break;
    default:
      setBridge(PIN_MOTOR_A_DIR, PIN_MOTOR_A_PWM, PIN_MOTOR_A_BRAKE, +1, pwmValue);
      setBridge(PIN_MOTOR_B_DIR, PIN_MOTOR_B_PWM, PIN_MOTOR_B_BRAKE, -1, pwmValue);
      break;
  }
}

void setBridge(uint8_t dirPin, uint8_t pwmPin, uint8_t brakePin, int sign, uint8_t pwmValue) {
  if (pwmValue == 0U) {
    analogWrite(pwmPin, 0);
    digitalWrite(brakePin, HIGH);
    return;
  }

  digitalWrite(brakePin, LOW);
  digitalWrite(dirPin, (sign >= 0) ? HIGH : LOW);
  analogWrite(pwmPin, pwmValue);
}

void releaseStepper() {
  analogWrite(PIN_MOTOR_A_PWM, 0);
  analogWrite(PIN_MOTOR_B_PWM, 0);
  digitalWrite(PIN_MOTOR_A_BRAKE, HIGH);
  digitalWrite(PIN_MOTOR_B_BRAKE, HIGH);
}

void stopController() {
  controllerEnabled = false;
  controlMode = MODE_IDLE;
  motorTestDirection = 0;
  demandedStepRate = 0.0f;
  appliedStepRate = 0.0f;
  appliedCoilPwm = 0;
  motorHoldingEnabled = false;
  releaseStepper();
}

void resetArmEstimate() {
  commandedArmSteps = 0;
  armAngleRad = 0.0f;
  armRateRadS = 0.0f;
  prevArmContinuousRad = 0.0f;
}

void startMotorTest(int direction) {
  controllerEnabled = false;
  controlMode = MODE_IDLE;
  motorTestDirection = (direction >= 0) ? 1 : -1;
  motorTestUntilMs = millis() + MOTOR_TEST_DURATION_MS;
  Serial.println((direction >= 0) ? F("Motor test forward.") : F("Motor test reverse."));
}

void printHelp() {
  Serial.println(F("Commands:"));
  Serial.println(F("  z : capture current pendulum position as 0 deg start"));
  Serial.println(F("  t : capture current position as 180 deg target"));
  Serial.println(F("  e : enable controller"));
  Serial.println(F("  x : disable controller"));
  Serial.println(F("  f : motor test forward"));
  Serial.println(F("  b : motor test reverse"));
  Serial.println(F("  r : reset estimated arm angle to zero"));
  Serial.println(F("  p : print status"));
  Serial.println(F("  ? : print help"));
}

void printStatus() {
  Serial.println(F("---- Status ----"));
  Serial.print(F("controllerEnabled = "));
  Serial.println(controllerEnabled ? F("true") : F("false"));
  Serial.print(F("encoderStartEdges = "));
  Serial.println(encoderStartEdges);
  Serial.print(F("pendulumPositionDeg = "));
  Serial.println(pendulumPositionRad * RAD_TO_DEG, 3);
  Serial.print(F("targetErrorDeg = "));
  Serial.println(pendulumAngleRad * RAD_TO_DEG, 3);
  Serial.print(F("pendulumRateRadS = "));
  Serial.println(pendulumRateRadS, 3);
  Serial.print(F("demandedStepRate = "));
  Serial.println(demandedStepRate, 3);
  Serial.print(F("armAngleDegEst = "));
  Serial.println(armAngleRad * RAD_TO_DEG, 3);
  Serial.print(F("armLimitDeg = +/-"));
  Serial.println(ARM_LIMIT_RAD * RAD_TO_DEG, 1);
  Serial.print(F("commandedArmSteps = "));
  Serial.println(commandedArmSteps);
  Serial.print(F("appliedStepRate = "));
  Serial.println(appliedStepRate, 3);
}

void handleSerial() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();

    switch (c) {
      case 'z':
      case 'Z':
      case 'd':
      case 'D':
        encoderStartEdges = readPendulumEdges() + ENCODER_HALF_TURN_EDGES;
        prevPendulumContinuousRad = -PENDULUM_DISPLAY_OFFSET_RAD;
        pendulumContinuousRad = -PENDULUM_DISPLAY_OFFSET_RAD;
        pendulumPositionRad = 0.0f;
        pendulumAngleRad = -PI;
        pendulumRateRadS = 0.0f;
        Serial.println(F("Captured current position as 0 deg start."));
        break;

      case 't':
      case 'T':
      case 'u':
      case 'U':
        encoderStartEdges = readPendulumEdges();
        prevPendulumContinuousRad = 0.0f;
        pendulumContinuousRad = 0.0f;
        pendulumPositionRad = PENDULUM_TARGET_RAD;
        pendulumAngleRad = 0.0f;
        pendulumRateRadS = 0.0f;
        Serial.println(F("Captured current position as 180 deg target."));
        break;

      case 'e':
      case 'E':
        controllerEnabled = true;
        controlMode = MODE_SWING_UP;
        Serial.println(F("Controller enabled."));
        break;

      case 'x':
      case 'X':
        stopController();
        Serial.println(F("Controller disabled."));
        break;

      case 'f':
      case 'F':
        startMotorTest(1);
        break;

      case 'b':
      case 'B':
        startMotorTest(-1);
        break;

      case 'r':
      case 'R':
        resetArmEstimate();
        Serial.println(F("Estimated arm angle reset."));
        break;

      case 'p':
      case 'P':
        printStatus();
        break;

      case '?':
        printHelp();
        break;

      default:
        break;
    }
  }
}

long readPendulumEdges() {
  noInterrupts();
  long edges = pendulumEncoderEdges;
  interrupts();
  return edges;
}

float wrapToTwoPi(float angleRad) {
  while (angleRad >= TWO_PI_F) {
    angleRad -= TWO_PI_F;
  }
  while (angleRad < 0.0f) {
    angleRad += TWO_PI_F;
  }
  return angleRad;
}

float wrapToPi(float angleRad) {
  while (angleRad > PI) {
    angleRad -= TWO_PI_F;
  }
  while (angleRad < -PI) {
    angleRad += TWO_PI_F;
  }
  return angleRad;
}

float constrainFloat(float value, float low, float high) {
  if (value < low) {
    return low;
  }
  if (value > high) {
    return high;
  }
  return value;
}

float signNoZero(float value) {
  if (value >= 0.0f) {
    return 1.0f;
  }
  return -1.0f;
}
