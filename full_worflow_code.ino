/*
  Integrated Platform + Grabber Workflow
  ======================================
  Workflow steps (with 2 s pause between each):
    1) Platform homes
    2) Grabber arm homes
    3) Grabber arm → 60°
    4) Grabber arm → 178°
    5) Platform moves forward (full travel)
    6) Grabber arm → 60°
    7) Platform homes
    8) Grabber arm homes

  NOTE ON PIN ASSIGNMENTS:
    • The original “platform” code used pins 2 & 3 for its two home-switch inputs.
    • The original “grabber” code also used pin 2 for encoder and pin 3 for its home-switch.
    • To avoid a hardware conflict, we have reassigned the grabber’s pins here:
        – GRABBER_ENC_PIN  = 18
        – GRABBER_HOME_PIN = 19
      Wire your grabber’s encoder output to pin 18 and its NC home switch to pin 19.
    • The platform still uses:
        – HOME1_PIN = 3
        – HOME2_PIN = 2
      for its two home-switches.

  ----------------------------------------------------------------------------
  This sketch pulls in both original codebases, reorganizes them into functions,
  and drives them in a simple state-machine (workflowStep 0..7). Each step
  blocks until completion, then uses safeDelay(2000) before proceeding.
  At any time, entering 'i' or '1' on the serial monitor triggers an emergency stop:
    – Platform stepping halts immediately (loops check emergency).
    – Grabber DAC is driven to 2048 (brake).
  Before each step, type 'n' to continue; at startup, type 's' to begin.
*/

// (1) COMMON #INCLUDES
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MCP4725.h>
#include <PID_v1.h>

// (2) PIN DEFINITIONS
// Platform (stepper on “both sides” + two home switches)
const int STEP_A_PIN   = 13;  // platform step pin A
const int STEP_B_PIN   = 12;  // platform step pin B
const int DIR_PIN      = 11;  // platform direction pin
const int HOME1_PIN    =  3;  // platform home switch 1 (active LOW)
const int HOME2_PIN    =  2;  // platform home switch 2 (active LOW)

// Linear Actuator
const int PWM1 = 10;
const int INA1 = 9;
const int INB1 = 8;

// Grabber arm (encoder, DAC driver, single home switch)
const uint8_t GRABBER_ENC_PIN   = 18;  // encoder → interrupt pin
const uint8_t GRABBER_HOME_PIN  = 19;  // grabber NC limit switch (active LOW)

const uint8_t DAC_ADDR          = 0x60; // I²C address of MCP4725 (grabber’s DAC)

// (3) PLATFORM MOTION PARAMETERS & GLOBALS
// pulses/rev & lead (both axes share these)
const unsigned long PPR        = 400UL;    // pulses per motor rev
const float           LEAD_MM  = 10.0;     // mm per rev
const int             STEPS_PER_MM = (int)(PPR / LEAD_MM);  // = 40 steps/mm

// distances: HOME-clear & travel (in mm)
const float           HOMING_CLEAR_IN   = 26.3;   // inches to clear before switch
const long            HOMING_CLEAR_MM   = (long)(HOMING_CLEAR_IN * 25.4 + 0.5);
const long            HOMING_CLEAR_STEPS = HOMING_CLEAR_MM * STEPS_PER_MM;

const float           TRAVEL_IN         = 25.0;   // inches total travel
const long            TRAVEL_MM         = (long)(TRAVEL_IN * 25.4 + 0.5);
const long            TRAVEL_STEPS      = TRAVEL_MM * STEPS_PER_MM;

// RPM settings (home vs travel)
const unsigned long   RPM_HOME       = 300UL;
const unsigned long   RPM_HOME_TRAP  = 950UL;  // trapezoidal homing
const unsigned long   RPM_TRAVEL     = 950UL;
const unsigned long   RPM_MAX_SAFE   = 1100UL;

// Computed half-pulse delays (µs)
unsigned long PULSE_US_HOME;        // slow homing
unsigned long PULSE_US_HOME_TRAP;   // trapezoidal homing
unsigned long PULSE_US_TRAVEL;      // travel

// “slow” vs “fast” delays for travel/trapezoid
unsigned long US_HOME_FAST, US_HOME_SLOW_START, US_HOME_SLOW_END;
unsigned long US_TRAVEL_FAST, US_TRAVEL_SLOW_START, US_TRAVEL_SLOW_END;

// ramp-profile breakpoints: accel/decel/cruise (in steps)
long homeAccelSteps, homeCruiseSteps, homeDecelSteps;
long accelSteps, cruiseSteps, decelSteps;

// runtime flags for platform
volatile bool emergency = false;    // if set, abort everything

// (4) GRABBER ARM GLOBALS
// Encoder & PID (grabber)
volatile long encoderCount = 0;    // updated in ISR
volatile int8_t motorDir = 1;      // +1 = forward/CW, −1 = reverse/CCW
long prevEncoderCount = 0;

// PID parameters for grabber motion
double Kp = 0.004, Ki = 0.0, Kd = 0.0021;
double pidInput, pidOutput, pidSetpoint;
PID    pid(&pidInput, &pidOutput, &pidSetpoint, Kp, Ki, Kd, DIRECT);

// Encoder & gearbox constants (grabber)
const double  ENC_CPR        = 100.0;   // counts per motor rev
const double  GEAR_RATIO     = 240.0;   // motor revs per output rev
const double  COUNTS_PER_DEG = (ENC_CPR * GEAR_RATIO) / 360.0;  // ≈66.6667 counts/deg

// Deadband (counts) to avoid chatter on direction flip
const long    DEADZONE_COUNTS = 80;

// DAC codes (grabber braking / ±2.5 V)
const uint16_t DAC_CODE_BRAKE  = 2048;  // ~2.5 V out = brake
const uint16_t DAC_CODE_POS2P5 = 2048;  // +2.5 V drive
const uint16_t DAC_CODE_NEG2P5 = 2048;  // −2.5 V drive

Adafruit_MCP4725 dac;  // the grabber’s DAC

// (5) WORKFLOW STATE MACHINE
// workflowStep: uint8_t 0..7
//   0 → platformHome()
//   1 → grabberHome()
//   2 → grabber → 60°
//   3 → grabber → 178°
//   4 → platformMoveForward()
//   5 → grabber → 60° (back)
//   6 → platformHome()   ← resets DIR=LOW before homing
//   7 → grabberHome()
//  ≥8 → done/idle
uint8_t workflowStep = 0;

//==============================================================================
// Utility: Check serial input for 'i' or '1' to trigger emergency stop
//==============================================================================
void checkForEmergencyInput() {
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'i' || c == '1') {
      Serial.println(F("[Emergency] Stop key received: braking immediately."));
      // Apply brake on grabber DAC
      dac.setVoltage(DAC_CODE_BRAKE, false);
      emergency = true;
    }
  }
}

//==============================================================================
// Utility: Delay in 50 ms slices, checking for emergency during wait
//==============================================================================
void safeDelay(unsigned long ms) {
  unsigned long start = millis();
  while (millis() - start < ms) {
    delay(50);
    checkForEmergencyInput();
    if (emergency) return;
  }
}

//==============================================================================
// (6) PLATFORM: FUNCTIONS FOR HOMING & TRAVEL
//==============================================================================

// (6.1) Compute half-pulse delay for trapezoidal homing at index i
unsigned long getHomeDelay(long i) {
  if (emergency) return 0xFFFFFFFFUL;  // abort if emergency
  if (i < 0 || i >= HOMING_CLEAR_STEPS) {
    return US_HOME_SLOW_START;
  }
  // Ramp up (0..homeAccelSteps-1)
  if (i < homeAccelSteps) {
    long deltaSigned = (long)US_HOME_SLOW_START - (long)US_HOME_FAST;
    long denom = homeAccelSteps - 1;
    if (denom < 1) denom = 1;
    long numer = (long)i * deltaSigned;
    long frac = numer / denom;  // 0..deltaSigned
    long result = (long)US_HOME_SLOW_START - frac;
    if (result < 1) { emergency = true; return 0xFFFFFFFFUL; }
    return (unsigned long)result;
  }
  long idxCruiseEnd = homeAccelSteps + homeCruiseSteps;
  // Cruise
  if (i < idxCruiseEnd) {
    if (US_HOME_FAST < 1) { emergency = true; return 0xFFFFFFFFUL; }
    return US_HOME_FAST;
  }
  // Ramp down
  long j = i - idxCruiseEnd;
  if (j < homeDecelSteps) {
    long denom = homeDecelSteps - 1;
    if (denom < 1) denom = 1;
    unsigned long delta = US_HOME_SLOW_END - US_HOME_FAST;
    unsigned long numer = (unsigned long)j * delta;
    unsigned long frac = numer / (unsigned long)denom;
    unsigned long result = US_HOME_FAST + frac;
    if (result < 1) { emergency = true; return 0xFFFFFFFFUL; }
    return result;
  }
  // Final fallback “slow”
  if (US_HOME_SLOW_START < 1) { emergency = true; return 0xFFFFFFFFUL; }
  return US_HOME_SLOW_START;
}

// (6.2) Perform trapezoidal homing on “both sides” (blocking)
void homeBothSides_trapezoidal() {
  bool homed1 = false, homed2 = false;
  bool firstHit = false;
  unsigned long firstHitMillis = 0;

  Serial.println(F("[Platform] Starting trapezoidal homing..."));
  // Precompute “fast” & “slow”
  US_HOME_FAST = PULSE_US_HOME_TRAP;
  US_HOME_SLOW_START = (PULSE_US_HOME_TRAP < 2) ? 2 : PULSE_US_HOME_TRAP * 2;
  US_HOME_SLOW_END   = US_HOME_SLOW_START;

  // Step through full clearance distance
  for (long i = 0; i < HOMING_CLEAR_STEPS; i++) {
    checkForEmergencyInput();
    if (emergency) return;
    if (homed1 && homed2) break;

    // Step both motors if not yet homed
    if (!homed1) digitalWrite(STEP_A_PIN, HIGH);
    if (!homed2) digitalWrite(STEP_B_PIN, HIGH);

    unsigned long delayUs = getHomeDelay(i);
    if (delayUs == 0xFFFFFFFFUL) return;

    delayMicroseconds(delayUs);
    if (!homed1) digitalWrite(STEP_A_PIN, LOW);
    if (!homed2) digitalWrite(STEP_B_PIN, LOW);
    delayMicroseconds(delayUs);

    // Check home switches (active LOW)
    if (!homed1 && digitalRead(HOME1_PIN) == LOW) {
      homed1 = true;
      Serial.println(F("[Platform] Side 1 homed"));
    }
    if (!homed2 && digitalRead(HOME2_PIN) == LOW) {
      homed2 = true;
      Serial.println(F("[Platform] Side 2 homed"));
    }

    // If one side hits, start 0.5 s timer for the other
    if (!firstHit && (homed1 || homed2)) {
      firstHit = true;
      firstHitMillis = millis();
    }
    if (firstHit && !(homed1 && homed2)) {
      if (millis() - firstHitMillis > 500) {
        Serial.println(F("[Platform] ERROR: second switch not reached in 0.5 s"));
        emergency = true;
        return;
      }
    }
  }

  if (!homed1 || !homed2) {
    Serial.println(F("[Platform] ERROR: homing incomplete"));
    emergency = true;
    return;
  }
  Serial.println(F("[Platform] Trapezoidal homing complete."));
}

// (6.3) Compute half-pulse delay for trapezoidal travel at index i
unsigned long getTravelDelay(long i) {
  if (emergency) return 0xFFFFFFFFUL;
  if (i < 0 || i >= TRAVEL_STEPS) {
    return US_TRAVEL_SLOW_START;
  }
  // Ramp up
  if (i < accelSteps) {
    long deltaSigned = (long)US_TRAVEL_SLOW_START - (long)US_TRAVEL_FAST;
    long denom = accelSteps - 1;
    if (denom < 1) denom = 1;
    long numer = (long)i * deltaSigned;
    long frac = numer / denom;  // 0..deltaSigned
    long result = (long)US_TRAVEL_SLOW_START - frac;
    if (result < 1) { emergency = true; return 0xFFFFFFFFUL; }
    return (unsigned long)result;
  }
  long idxCruiseEnd = accelSteps + cruiseSteps;
  // Cruise
  if (i < idxCruiseEnd) {
    if (US_TRAVEL_FAST < 1) { emergency = true; return 0xFFFFFFFFUL; }
    return US_TRAVEL_FAST;
  }
  // Ramp down
  long j = i - idxCruiseEnd;
  if (j < decelSteps) {
    long denom = decelSteps - 1;
    if (denom < 1) denom = 1;
    unsigned long delta = US_TRAVEL_SLOW_END - US_TRAVEL_FAST;
    unsigned long numer = (unsigned long)j * delta;
    unsigned long frac = numer / (unsigned long)denom;
    unsigned long result = US_TRAVEL_FAST + frac;
    if (result < 1) { emergency = true; return 0xFFFFFFFFUL; }
    return result;
  }
  // Final fallback
  if (US_TRAVEL_SLOW_START < 1) { emergency = true; return 0xFFFFFFFFUL; }
  return US_TRAVEL_SLOW_START;
}

// (6.4) Perform full-length trapezoidal travel on “both sides” (blocking)
void stepBothWithTrapezoid() {
  Serial.println(F("[Platform] Starting full forward travel..."));
  for (long i = 0; i < TRAVEL_STEPS; i++) {
    checkForEmergencyInput();
    if (emergency) return;

    unsigned long delayUs = getTravelDelay(i);
    if (delayUs == 0xFFFFFFFFUL) return;

    digitalWrite(STEP_A_PIN, HIGH);
    digitalWrite(STEP_B_PIN, HIGH);
    delayMicroseconds(delayUs);
    digitalWrite(STEP_A_PIN, LOW);
    digitalWrite(STEP_B_PIN, LOW);
    delayMicroseconds(delayUs);
  }
  Serial.println(F("[Platform] Forward travel complete."));
}

// (6.5) “Public” wrappers for the platform steps
void platformHome() {
  digitalWrite(DIR_PIN, LOW);       // ensure homing direction is reverse
  homeBothSides_trapezoidal();
}
void platformMoveForward() {
  digitalWrite(DIR_PIN, HIGH);      // forward direction
  stepBothWithTrapezoid();
}

// (7) GRABBER: FUNCTIONS FOR HOMING & SINGLE‐MOVE

// (7.1) ISR for the grabber encoder
void encoderISR() {
  encoderCount += motorDir;
}

// (7.2) Home the grabber: spin CCW (motorDir = –1) until home switch opens,
// then zero the encoder exactly as in the original code, and restore motorDir=+1
void grabberHome() {
  detachInterrupt(digitalPinToInterrupt(GRABBER_ENC_PIN));
  pinMode(GRABBER_HOME_PIN, INPUT_PULLUP);
  Serial.println(">> HOMING: CCW (motorDir = -1) until switch opens...");
  motorDir = -1;

  // Drive CCW at low voltage until HOME_PIN reads LOW (switch opens)
  while (digitalRead(GRABBER_HOME_PIN) != LOW) {
    checkForEmergencyInput();
    if (emergency) return;
    // Send “small negative” drive. Vout ~0 V => code = 0 (slow CCW).
    dac.setVoltage(1638, false);
    delay(5);
  }

  // Immediately brake
  dac.setVoltage(DAC_CODE_BRAKE, false);
  delay(50);

  // Zero encoder count
  noInterrupts();
    encoderCount = 0;
  interrupts();
  Serial.println(">> HOMING complete. Encoder cleared to 0.\n");

}

// (7.3) Perform one “position‐tracking” move using original logic (blocking):
//        from current encoderCount to targetDeg with overrideDeg, totalTimeSec
void doGrabberStep(double targetDeg, double overrideDeg, double totalTimeSec) {
  // 1) Record start angle
  attachInterrupt(digitalPinToInterrupt(GRABBER_ENC_PIN), encoderISR, RISING);
  noInterrupts();
    long currCount = encoderCount;
  interrupts();
  double startDeg = currCount / COUNTS_PER_DEG;

  // 2) Initialize PID variables & direction
  prevEncoderCount = encoderCount;
  motorDir = (targetDeg >= startDeg) ? +1 : -1;  // direction for this step

  unsigned long stepStartTime   = millis();
  bool waitingForStop        = false;
  unsigned long stopDetectedTime  = 0;

  Serial.print(F("[Grabber] Moving from "));
  Serial.print(startDeg, 1);
  Serial.print(F("° → "));
  Serial.print(targetDeg, 1);
  Serial.print(F("°  (override @ "));
  Serial.print(overrideDeg, 1);
  Serial.println(F("°)..."));

  while (true) {
    checkForEmergencyInput();
    if (emergency) {
      dac.setVoltage(DAC_CODE_BRAKE, false);
      return;
    }

    // Compute time fraction
    double elapsedSec = (millis() - stepStartTime) / 1000.0;
    double s = elapsedSec / totalTimeSec;
    if (s > 1.0) s = 1.0;

    // Quintic S‐curve setpoint θ_d
    double deltaDeg = targetDeg - startDeg;
    double theta_d  = startDeg + deltaDeg * (10.0 * pow(s, 3)
                                            - 15.0 * pow(s, 4)
                                            +  6.0 * pow(s, 5));
    pidSetpoint = theta_d * COUNTS_PER_DEG;

    // Read encoder
    noInterrupts();
      currCount = encoderCount;
    interrupts();
    pidInput = (double)currCount;

    // Check override
    long overrideCount = (long)lround(overrideDeg * COUNTS_PER_DEG);
    bool atOverride = false;
    if (deltaDeg >= 0.0 && currCount >= overrideCount) {
      // Forward override: +2.5 V
      dac.setVoltage(DAC_CODE_POS2P5, false);
      atOverride = true;
    }
    else if (deltaDeg < 0.0 && currCount <= overrideCount) {
      // Reverse override: −2.5 V
      dac.setVoltage(DAC_CODE_NEG2P5, false);
      atOverride = true;
    }

    if (atOverride) {
      long deltaCounts = currCount - prevEncoderCount;
      // If motor stopped (deltaCounts == 0), start 2 s timer
      if (!waitingForStop) {
        if (deltaCounts == 0) {
          waitingForStop   = true;
          stopDetectedTime = millis();
        }
      }
      else {
        if (millis() - stopDetectedTime >= 500UL) {
          // 2 s elapsed → finish this move
          break;
        }
      }
      // --- Original override debug print ---
      Serial.print(F("t= "));
      Serial.print(elapsedSec, 2);
      Serial.print(F("  at override"));
      Serial.print(F("  Pos= "));
      Serial.print((pidInput / COUNTS_PER_DEG), 1);
      Serial.print(F(" deg  deltaCounts= "));
      Serial.print(deltaCounts);
      Serial.print(F("  mDir= "));
      Serial.println(motorDir);
      // ---------------------------------------
      prevEncoderCount = currCount;
      delay(25);
      continue;
    }

    // Normal PID control
    if (pid.Compute()) {
      int8_t desiredDir = (pidOutput >= 0.0) ? +1 : -1;
      long errorCounts  = lround(fabs(pidSetpoint - pidInput));
      long deltaCounts  = currCount - prevEncoderCount;

      // Flip direction if needed and motor is stopped
      if (desiredDir != motorDir) {
        if (deltaCounts == 0 && errorCounts > DEADZONE_COUNTS) {
          motorDir = desiredDir;
        }
      }
      // Map pidOutput (–5..+5) to 12‐bit DAC (0..4095)
      double clippedV = constrain(pidOutput, -5.0, +5.0);
      uint16_t dacCode = (uint16_t)lround((clippedV + 5.0) / 10.0 * 4095.0);
      dacCode = constrain(dacCode, 0, 4095);
      dac.setVoltage(dacCode, false);
    }

    // --- Original per-iteration debug print ---
    long deltaCounts = currCount - prevEncoderCount;
    Serial.print(F("t= "));
    Serial.print(elapsedSec, 2);
    Serial.print(F("  θ_d= "));
    Serial.print((theta_d), 1);
    Serial.print(F(" deg   Pos= "));
    Serial.print((pidInput / COUNTS_PER_DEG), 1);
    Serial.print(F(" deg   δCounts= "));
    Serial.print(deltaCounts);
    Serial.print(F("   Err= "));
    Serial.print(((pidSetpoint - pidInput) / COUNTS_PER_DEG), 1);
    Serial.print(F(" deg   Out(V)= "));
    Serial.print(pidOutput, 2);
    Serial.print(F("   mDir= "));
    Serial.println(motorDir);
    // -----------------------------------------

    prevEncoderCount = currCount;
    delay(25);
  }

  // Final brake
  dac.setVoltage(DAC_CODE_BRAKE, false);
  Serial.println(F("[Grabber] Move complete, brake applied.\n"));
}

// (7.4) Convenience wrappers for our three grabber moves
void grabberMoveTo60deg()     { doGrabberStep( 60.0,  59.0, 3.0); }
void grabberMoveTo178deg()    { doGrabberStep(178.0, 170.0, 5.0); }
void grabberMoveBackTo60deg() { doGrabberStep( 60.0,  65.0, 5.0); }

// (8) SETUP
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println(F("Integrated workflow ready. Press '1' or 'i' at any time for emergency stop."));

  // 1) Set pin modes before driving them:
  pinMode(PWM1, OUTPUT);
  pinMode(INA1, OUTPUT);
  pinMode(INB1, OUTPUT);

  // 2) Ensure they start LOW so the driver is disabled at reset:
  digitalWrite(PWM1, LOW);
  digitalWrite(INA1, LOW);
  digitalWrite(INB1, LOW);

  // (8.1) PLATFORM: pinMode & initial checks
  pinMode(STEP_A_PIN, OUTPUT);
  pinMode(STEP_B_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  digitalWrite(STEP_A_PIN, LOW);
  digitalWrite(STEP_B_PIN, LOW);
  digitalWrite(DIR_PIN, LOW); // ← DIR starts LOW (reverse)

  pinMode(HOME1_PIN, INPUT);
  pinMode(HOME2_PIN, INPUT);

  // Compute half‐pulse delays
  PULSE_US_HOME      = 30000000UL / (PPR * RPM_HOME);
  PULSE_US_HOME_TRAP = 30000000UL / (PPR * RPM_HOME_TRAP);
  PULSE_US_TRAVEL    = 30000000UL / (PPR * RPM_TRAVEL);

  // Safety checks
  if (RPM_HOME > RPM_MAX_SAFE || RPM_HOME_TRAP > RPM_MAX_SAFE || RPM_TRAVEL > RPM_MAX_SAFE
      || PULSE_US_HOME == 0 || PULSE_US_HOME_TRAP == 0 || PULSE_US_TRAVEL == 0) {
    Serial.println(F("[Setup] ERROR: One requested RPM is out of bounds."));
    emergency = true;
  }

  // Compute “slow”/“fast” & ramp breakpoints
  if (!emergency) {
    // Travel
    US_TRAVEL_FAST       = PULSE_US_TRAVEL;
    US_TRAVEL_SLOW_START = (PULSE_US_TRAVEL < 2) ? 2 : PULSE_US_TRAVEL * 2;
    US_TRAVEL_SLOW_END   = US_TRAVEL_SLOW_START;
    long rawTravel = TRAVEL_STEPS / 10;
    if (rawTravel < 2) rawTravel = 2;
    accelSteps = rawTravel;
    decelSteps = rawTravel;
    cruiseSteps = TRAVEL_STEPS - (accelSteps + decelSteps);
    if (cruiseSteps < 0) cruiseSteps = 0;

    // Homing
    US_HOME_FAST        = PULSE_US_HOME;
    US_HOME_SLOW_START  = (PULSE_US_HOME < 2) ? 2 : PULSE_US_HOME * 2;
    US_HOME_SLOW_END    = US_HOME_SLOW_START;
    long rawHome = HOMING_CLEAR_STEPS / 10;
    if (rawHome < 2) rawHome = 2;
    homeAccelSteps = rawHome;
    homeDecelSteps = rawHome;
    homeCruiseSteps = HOMING_CLEAR_STEPS - (homeAccelSteps + homeDecelSteps);
    if (homeCruiseSteps < 0) homeCruiseSteps = 0;
  }

  // (8.2) GRABBER: encoder interrupt + DAC + PID setup
  pinMode(GRABBER_ENC_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(GRABBER_ENC_PIN), encoderISR, RISING);

  pinMode(GRABBER_HOME_PIN, INPUT_PULLUP);

  // Initialize DAC
  Wire.begin();
  if (!dac.begin(DAC_ADDR)) {
    Serial.println(F("[Setup] ERROR: MCP4725 not found!"));
    emergency = true;
    while (1);
  }

  // PID
  pid.SetMode(AUTOMATIC);
  pid.SetOutputLimits(-5.0, +5.0);
  pid.SetSampleTime(25);  // 25 ms → 40 Hz

  // Ensure encoderCount = 0, motorDir = +1 to start
  noInterrupts();
    encoderCount = 0;
  interrupts();
  motorDir = 1;
  prevEncoderCount = 0;

  // (8.3) Prompt user to start the program
  Serial.println(F("\n=== Setup complete. Enter 's' to start workflow. ==="));
  while (true) {
    if (Serial.available()) {
      char c = Serial.read();
      if (c == 's') break;
      if (c == 'i' || c == '1') { emergency = true; break; }
    }
  }
  if (!emergency) Serial.println(F("Starting workflow..."));
}

// (9) MAIN LOOP (drives the 8‐step workflow)
void loop() {
  if (emergency) {
    // If any error occurred, idle with brake applied on grabber
    dac.setVoltage(DAC_CODE_BRAKE, false);
    return;
  }

  switch (workflowStep) {
    case 0:
      Serial.println(F("\n>> Enter 'n' to begin Step 0: Platform homing"));
      while (true) {
        if (Serial.available()) {
          char c = Serial.read();
          if (c == 'i' || c == '1') { emergency = true; return; }
          if (c == 'n') break;
        }
      }
      Serial.println(F("[Step 0] Platform homing"));
      platformHome();            // ← sets DIR = LOW before homing
      safeDelay(50);
      workflowStep++;
      break;

    case 1:
      Serial.println(F("\n>> Enter 'n' to begin Step 1: Grabber homing"));
      while (true) {
        if (Serial.available()) {
          char c = Serial.read();
          if (c == 'i' || c == '1') { emergency = true; return; }
          if (c == 'n') break;
        }
      }
      Serial.println(F("[Step 1] Grabber homing"));
      grabberHome();
      safeDelay(50);
      digitalWrite(INA1, LOW);
      digitalWrite(INB1, HIGH);
      analogWrite(PWM1, 255);
      Serial.println("extending");
      delay(4000);
      workflowStep++;
      break;

    case 2:
      Serial.println(F("\n>> Enter 'n' to begin Step 2: Grabber → 60°"));
      while (true) {
        if (Serial.available()) {
          char c = Serial.read();
          if (c == 'i' || c == '1') { emergency = true; return; }
          if (c == 'n') break;
        }
      }
      Serial.println(F("[Step 2] Grabber → 60°"));
      grabberMoveTo60deg();
      safeDelay(50);
      workflowStep++;
      break;

    case 3:
      Serial.println(F("\n>> Enter 'n' to begin Step 3: Grabber → 178°"));
      while (true) {
        if (Serial.available()) {
          char c = Serial.read();
          if (c == 'i' || c == '1') { emergency = true; return; }
          if (c == 'n') break;
        }
      }
      Serial.println(F("[Step 3] Grabber → 178°"));
      grabberMoveTo178deg();
      safeDelay(50);
      workflowStep++;
      break;

    case 4:
      Serial.println(F("\n>> Enter 'n' to begin Step 4: Platform → full forward"));
      while (true) {
        if (Serial.available()) {
          char c = Serial.read();
          if (c == 'i' || c == '1') { emergency = true; return; }
          if (c == 'n') break;
        }
      }
      Serial.println(F("[Step 4] Platform → full forward"));
      platformMoveForward();
      safeDelay(50);
      digitalWrite(INA1, HIGH);
      digitalWrite(INB1, LOW);
      analogWrite(PWM1, 255);
      Serial.println("retracting");
      delay(4000);
      workflowStep++;
      break;

    case 5:
      Serial.println(F("\n>> Enter 'n' to begin Step 5: Grabber → 60° (back)"));
      while (true) {
        if (Serial.available()) {
          char c = Serial.read();
          if (c == 'i' || c == '1') { emergency = true; return; }
          if (c == 'n') break;
        }
      }
      Serial.println(F("[Step 5] Grabber → 60° (back)"));
      grabberMoveBackTo60deg();
      safeDelay(50);
      workflowStep++;
      break;

    case 6:
      Serial.println(F("\n>> Enter 'n' to begin Step 6: Platform homing (again)"));
      while (true) {
        if (Serial.available()) {
          char c = Serial.read();
          if (c == 'i' || c == '1') { emergency = true; return; }
          if (c == 'n') break;
        }
      }
      Serial.println(F("[Step 6] Platform homing (again)"));
      platformHome();            // ← sets DIR = LOW before homing again
      safeDelay(2000);
      workflowStep++;
      break;

    case 7:
      Serial.println(F("\n>> Enter 'n' to begin Step 7: Grabber homing (end)"));
      while (true) {
        if (Serial.available()) {
          char c = Serial.read();
          if (c == 'i' || c == '1') { emergency = true; return; }
          if (c == 'n') break;
        }
      }
      Serial.println(F("[Step 7] Grabber homing (end)"));
      grabberHome();
      workflowStep++;
      break;

    default:
      // workflowStep ≥ 8 → done/idle (brake remains applied)
      dac.setVoltage(DAC_CODE_BRAKE, false);
      break;
  }
}
