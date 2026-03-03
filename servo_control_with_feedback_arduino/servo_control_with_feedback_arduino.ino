/*
===============================================================================
Title:        servo_control_with_feedback
Author:       Alejandro Alonso Puig + ChatGPT
License:      Apache 2.0
-------------------------------------------------------------------------------
Description:

Generic servo control experiment with:

- Trapezoidal motion profile (position, vmax, acceleration)
- PWM command output to a standard hobby servo
- Real position feedback from the servo internal potentiometer (ADC)
- Telemetry over USB (Serial) for analysis and tuning

-------------------------------------------------------------------------------
SERVO SPEED PARAMETER (IMPORTANT)

This sketch uses one physical characterization parameter:

    SERVO_MAX_SPEED_DEGPS   [deg/s]

It represents the *maximum real angular speed* the servo can achieve under
the current supply voltage and load conditions.

Example conversions from datasheet-style specs:

  HS-805BB @6V
  0.14 s / 60°  ->  60 / 0.14  = 428.6 deg/s

  Futaba S3003 @4.8V
  0.23 s / 60°  ->  60 / 0.23  = 261.0 deg/s   (DEFAULT USED HERE)

If you change the servo model, the supply voltage, or mechanical conditions
that affect speed, you MUST update SERVO_MAX_SPEED_DEGPS accordingly.

-------------------------------------------------------------------------------
USB TELEMETRY COLUMNS (printed each loop)

Target | Cmd | Meas | Err | V | V% | A% | PWM | ADC

- Target : target angle from the target potentiometer (deg)
- Cmd    : commanded reference angle from the motion profile (deg)
- Meas   : measured real angle from servo internal potentiometer (deg)
- Err    : tracking error = Cmd - Meas (deg)
- V      : profile instantaneous velocity (deg/s)
- V%     : user knob for vmax limit (1..100 %) of SERVO_MAX_SPEED_DEGPS
- A%     : user knob for accel limit (1..100 %) of ACCEL_MAX_DEGPS2
- PWM    : microseconds sent to the servo (us)
- ADC    : raw averaged ADC reading from the feedback pin (0..1023)

===============================================================================
*/

#include <Arduino.h>
#include <Servo.h>

// ============================ USER CONFIGURATION ============================

// ---- Pins ----
// SERVO_PIN      : PWM output to servo signal wire
// POT_TARGET_PIN : user knob for target angle (0..180°)
// POT_VMAX_PIN   : user knob for vmax% (1..100%)
// POT_ACCEL_PIN  : user knob for accel% (1..100%)
// POT_FB_PIN     : analog input connected to servo internal potentiometer
#define SERVO_PIN        9
#define POT_TARGET_PIN   A0
#define POT_VMAX_PIN     A1
#define POT_ACCEL_PIN    A2
#define POT_FB_PIN       A4

#define BAUDRATE         115200

// ---- Control loop timing ----
// Typical analog hobby servos expect a ~20 ms refresh frame.
// This sketch updates at that same pace.
#define LOOP_INTERVAL_MS 20

// ---- Servo PWM calibration ----
// These define how the 0..180° command maps to microseconds.
// Adjust to match YOUR servo endpoints safely.
//
// SERVO_CENTER_US  : pulse width for servo center (usually ~1500 us)
// SERVO_HALFSPAN_US: +/- range around center that maps to 0..180°
//                   (keep conservative to avoid hitting hard stops)
#define SERVO_CENTER_US     1500
#define SERVO_HALFSPAN_US    860

#define PWM_MIN_US (SERVO_CENTER_US - SERVO_HALFSPAN_US)
#define PWM_MAX_US (SERVO_CENTER_US + SERVO_HALFSPAN_US)

// ---- Feedback ADC calibration ----
// You must measure the ADC value produced by the internal potentiometer at
// known angles to calibrate the mapping ADC -> degrees.
//
// FB_ADC_AT_0_DEG   : ADC reading when servo is at 0°
// FB_ADC_AT_180_DEG : ADC reading when servo is at 180°
static const float FB_ADC_AT_0_DEG   =  91.0f;
static const float FB_ADC_AT_180_DEG = 376.0f;

// ---- Physical servo characterization ----
// Real maximum angular speed of the actuator (deg/s) for the servo model
// and supply voltage used.
//
// DEFAULT: Futaba S3003 @4.8V
// 0.23 s / 60°  ->  60 / 0.23 = 261 deg/s
//
// If you use another servo or change supply voltage, update this value.
#define SERVO_MAX_SPEED_DEGPS 261.0f

// ---- Acceleration reference (100% = this value) ----
// This is a chosen design limit for acceleration used by the software profile.
// It does NOT come from the datasheet. Pick a value that gives a useful range.
//
// Higher values -> more aggressive acceleration ramps (snappier motion).
// Lower values  -> gentler ramps (smoother, slower response).
#define ACCEL_MAX_DEGPS2  800.0f

// ---- Percent mapping ----
// User knobs for Vmax and Accel are expressed as integer percent.
#define PCT_MIN  1
#define PCT_MAX  100

// ---- ADC and filtering ----
#define ADC_SCALE   1023.0f   // 10-bit ADC
#define FB_SAMPLES  6         // samples used for feedback averaging

// ============================ GLOBAL STATE ==================================

Servo s;                      // Servo object (Arduino Servo library)

float cmdDeg = 0.0f;          // Commanded reference angle from profile (deg)
float vDegps = 0.0f;          // Profile instantaneous velocity (deg/s)

unsigned long lastTick = 0;   // last loop time (ms)

// Precomputed slope for ADC → degrees conversion.
// Using two-point calibration: adc -> deg = (adc - adc0) * slope
static const float FB_DEG_PER_ADC =
  180.0f / (FB_ADC_AT_180_DEG - FB_ADC_AT_0_DEG);

// ============================ UTILITY FUNCTIONS =============================

// Clamp float to [lo..hi].
// Keeps computations safe and prevents bad inputs from exploding.
static inline float clampf(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

// Read an analog pin multiple times and return the average.
// This reduces ADC noise cheaply without extra libraries.
float readAveragedADC(uint8_t pin, int samples) {
  long sum = 0;
  for (int i = 0; i < samples; i++) {
    sum += analogRead(pin);
  }
  return (float)sum / samples;
}

// Convert the target potentiometer ADC reading (0..1023) to degrees (0..180).
// This assumes the pot is wired as a divider over the full ADC range.
float targetDegFromPot(float adc) {
  float deg = (adc / ADC_SCALE) * 180.0f;
  return clampf(deg, 0.0f, 180.0f);
}

// Convert servo internal potentiometer ADC reading into degrees using
// the two calibration points (ADC at 0° and ADC at 180°).
float feedbackDegFromAdc(float adc) {
  float deg = (adc - FB_ADC_AT_0_DEG) * FB_DEG_PER_ADC;
  return clampf(deg, 0.0f, 180.0f);
}

// Convert desired angle (deg) to PWM microseconds.
// Linear mapping between [0..180] and [PWM_MIN_US..PWM_MAX_US].
// Note: many servos are not perfectly linear, but this is a good baseline.
int pwmUsFromDeg(float deg) {
  deg = clampf(deg, 0.0f, 180.0f);
  float us = PWM_MIN_US + (deg / 180.0f) * (PWM_MAX_US - PWM_MIN_US);
  return (int)(us + 0.5f);   // round to nearest int
}

// Map ADC (0..1023) to integer percent (1..100) using full knob travel.
// This avoids the “only first 10% works” feeling: the knob always spans 1..100.
int percentFromAdc(float adc) {
  float t = clampf(adc / ADC_SCALE, 0.0f, 1.0f);                 // 0..1
  int pct = (int)(PCT_MIN + t * (PCT_MAX - PCT_MIN) + 0.5f);     // rounded
  if (pct < PCT_MIN) pct = PCT_MIN;
  if (pct > PCT_MAX) pct = PCT_MAX;
  return pct;
}

// Convert Vmax% (1..100) into a physical speed limit in deg/s.
// The key idea: 100% means "use the real maximum speed of this servo".
// So the full knob range is meaningful for any servo speed.
float vmaxDegpsFromPercent(int vmaxPct) {
  float t = (float)vmaxPct / 100.0f;                             // 0.01..1.00
  float v = SERVO_MAX_SPEED_DEGPS * t;                           // scaled
  // Safety clamp, ensures > 0 and never exceeds the real maximum.
  return clampf(v, SERVO_MAX_SPEED_DEGPS * 0.01f, SERVO_MAX_SPEED_DEGPS);
}

// Convert Accel% (1..100) into acceleration limit deg/s².
// This uses ACCEL_MAX_DEGPS2 as a software reference (100%).
float accelDegps2FromPercent(int accelPct) {
  float t = (float)accelPct / 100.0f;                            // 0.01..1.00
  float a = ACCEL_MAX_DEGPS2 * t;                                // scaled
  // Keep strictly positive (avoids corner cases in sqrt and stop math).
  return max(a, ACCEL_MAX_DEGPS2 * 0.01f);
}

// Fixed-width telemetry printing.
// dtostrf() formats floats into char buffers with fixed width and decimals.
// This keeps columns aligned even when values change digits.
void printRowFixed(float targetDeg, float cmdDeg, float measDeg, float errDeg,
                   float vDegps, int vmaxPct, int accelPct, int pwmUs, float adcFb)
{
  char a[12], b[12], c[12], d[12], e[12], f[12];

  // width 6, 1 decimal -> e.g. "  12.3"
  dtostrf(targetDeg, 6, 1, a);
  dtostrf(cmdDeg,    6, 1, b);
  dtostrf(measDeg,   6, 1, c);
  dtostrf(errDeg,    6, 1, d);
  dtostrf(vDegps,    6, 1, e);
  dtostrf(adcFb,     6, 1, f);

  // Target |   Cmd |  Meas |   Err |     V |  V% |  A% |  PWM |   ADC
  Serial.print(a); Serial.print(" | ");
  Serial.print(b); Serial.print(" | ");
  Serial.print(c); Serial.print(" | ");
  Serial.print(d); Serial.print(" | ");
  Serial.print(e); Serial.print(" | ");

  // V% integer, padded to width 3 (right-aligned)
  if (vmaxPct < 100) Serial.print(' ');
  if (vmaxPct < 10)  Serial.print(' ');
  Serial.print(vmaxPct); Serial.print("% | ");

  // A% integer, padded to width 3 (right-aligned)
  if (accelPct < 100) Serial.print(' ');
  if (accelPct < 10)  Serial.print(' ');
  Serial.print(accelPct); Serial.print("% | ");

  // PWM microseconds, padded to 4-ish
  if (pwmUs < 1000) Serial.print(' ');
  Serial.print(pwmUs); Serial.print(" | ");

  // Raw feedback ADC (averaged), fixed width float
  Serial.println(f);
}

// ============================ MOTION PROFILE ================================

// Motion profile update (executed every loop):
//
// Inputs:
//   targetDeg     : desired final angle (deg)
//   vmaxDegps     : speed limit (deg/s) derived from V% and SERVO_MAX_SPEED_DEGPS
//   accelDegps2   : acceleration limit (deg/s²) derived from A% and ACCEL_MAX_DEGPS2
//   dt            : loop time step (s)
//
// State (updated):
//   cmdDeg        : commanded reference position (deg)
//   vDegps        : profile velocity (deg/s)
//
// Behaviour summary:
//
// - Computes remaining distance to target.
// - Computes vStop: maximum speed that still allows stopping within remaining distance.
// - Allowed speed = min(vmax, vStop).
// - Accelerates current speed toward allowed speed by accel*dt.
// - Advances cmdDeg by v*dt, and snaps to target if overshooting.
//
void updateProfile(float targetDeg,
                   float vmaxDegps,
                   float accelDegps2,
                   float dt)
{
  // Distance to go (deg). Sign indicates direction.
  float dist = targetDeg - cmdDeg;

  // Direction (+1 or -1) used to keep velocity sign consistent.
  float dir  = (dist >= 0.0f) ? 1.0f : -1.0f;

  // Absolute distance remaining (deg) used for stop calculations.
  float dabs = fabs(dist);

  // If we are extremely close, snap to the target and stop.
  // This avoids jitter caused by ADC noise and small oscillations.
  if (dabs < 0.2f) {
    cmdDeg = targetDeg;
    vDegps = 0.0f;
    return;
  }

  // vStop (deg/s): maximum speed that still allows stopping within dabs
  // assuming constant deceleration magnitude = accelDegps2.
  //
  // From kinematics: v^2 = 2*a*s  -> v = sqrt(2*a*s)
  float vStop = sqrtf(2.0f * accelDegps2 * dabs);

  // Allowed speed is limited by:
  // - user-selected vmax (already within real servo max)
  // - remaining distance stop constraint (vStop)
  float vAllowed = min(vmaxDegps, vStop);

  // Current speed magnitude (deg/s), ignoring sign.
  float vMag = fabs(vDegps);

  // Increase speed magnitude by accel*dt (simple acceleration ramp).
  // (Deceleration happens automatically because vAllowed shrinks near target.)
  vMag += accelDegps2 * dt;

  // Cap speed to the currently allowed maximum.
  if (vMag > vAllowed) vMag = vAllowed;

  // Restore direction sign.
  vDegps = dir * vMag;

  // Integrate position by v*dt.
  float delta = vDegps * dt;

  // If the computed step would overshoot the target, snap and stop.
  if (fabs(delta) > dabs) {
    cmdDeg = targetDeg;
    vDegps = 0.0f;
  } else {
    cmdDeg += delta;
  }
}

// ============================ SETUP =========================================

void setup() {

  // Start USB serial for telemetry.
  Serial.begin(BAUDRATE);

  // Small delay for serial monitor to attach reliably.
  delay(500);

  // Attach servo with calibrated min/max pulse limits.
  s.attach(SERVO_PIN, PWM_MIN_US, PWM_MAX_US);

  // Initialize cmdDeg from measured position so the servo does not jump at boot.
  // We read the internal potentiometer (feedback) and start from that angle.
  float adc0   = readAveragedADC(POT_FB_PIN, FB_SAMPLES);
  float fbDeg0 = feedbackDegFromAdc(adc0);

  cmdDeg = fbDeg0;       // start command from actual measured angle
  vDegps = 0.0f;         // start at rest

  // Send initial PWM command consistent with cmdDeg.
  s.writeMicroseconds(pwmUsFromDeg(cmdDeg));

  // Start loop timer.
  lastTick = millis();

  // Print header and a separator line.
  Serial.println("servo_control_with_feedback");
  Serial.println("Target |   Cmd |  Meas |   Err |     V |  V% |  A% |  PWM |   ADC");
  Serial.println("------------------------------------------------------------------------");
}

// ============================ LOOP ==========================================

void loop() {

  // Enforce fixed loop interval (non-blocking timing).
  unsigned long now = millis();
  if (now - lastTick < LOOP_INTERVAL_MS) return;

  // Compute dt in seconds from elapsed milliseconds.
  float dt = (now - lastTick) / 1000.0f;
  lastTick = now;

  // ---- Read user controls (averaged to reduce ADC noise) ----
  float adcTarget = readAveragedADC(POT_TARGET_PIN, 4);   // target knob
  float adcVmax   = readAveragedADC(POT_VMAX_PIN,   4);   // vmax knob
  float adcAccel  = readAveragedADC(POT_ACCEL_PIN,  4);   // accel knob

  // Convert target knob to degrees.
  float targetDeg = targetDegFromPot(adcTarget);

  // Convert vmax and accel knobs to integer percent (1..100).
  int vmaxPct  = percentFromAdc(adcVmax);
  int accelPct = percentFromAdc(adcAccel);

  // Convert percent values to physical limits for the motion profile.
  //
  // vmaxDegps   : 1..100% of the REAL servo max speed (SERVO_MAX_SPEED_DEGPS)
  // accelDegps2 : 1..100% of chosen reference accel (ACCEL_MAX_DEGPS2)
  float vmaxDegps   = vmaxDegpsFromPercent(vmaxPct);
  float accelDegps2 = accelDegps2FromPercent(accelPct);

  // ---- Update motion profile (Cmd and V are updated here) ----
  updateProfile(targetDeg, vmaxDegps, accelDegps2, dt);

  // ---- Send command to servo ----
  int pwmUs = pwmUsFromDeg(cmdDeg);
  s.writeMicroseconds(pwmUs);

  // ---- Read feedback from servo internal potentiometer ----
  float adcFb   = readAveragedADC(POT_FB_PIN, FB_SAMPLES);
  float measDeg = feedbackDegFromAdc(adcFb);

  // Tracking error (deg): commanded profile angle minus measured real angle.
  float err = cmdDeg - measDeg;

  // ---- Telemetry (fixed width, aligned columns) ----
  printRowFixed(targetDeg, cmdDeg, measDeg, err, vDegps, vmaxPct, accelPct, pwmUs, adcFb);
}
