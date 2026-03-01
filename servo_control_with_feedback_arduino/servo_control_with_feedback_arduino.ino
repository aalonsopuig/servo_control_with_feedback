/*
===============================================================================
Title:        servo_control_with_feedback_arduino
Date:         2026-03-01
Author:       Alejandro Alonso Puig + ChatGPT
License:      Apache 2.0 License
-------------------------------------------------------------------------------
Description:
This sketch drives an RC servo using a trapezoidal motion profile generated in
software from three user potentiometers:
  - Target angle (deg)   from A0
  - Max speed (deg/s)    from A1
  - Acceleration (deg/s²)from A2

Additionally, it reads the servo's internal feedback potentiometer (wiper)
through an external RC filter (1k + 4.7uF) connected to A4.

Key points:
- On startup, the servo command is initialized to the *measured* feedback angle,
  so the servo does not jump to an arbitrary angle.
- The sketch logs both the commanded position (the internal reference updated
  each cycle) and the measured position, plus error = commanded - measured.

Goal:
Empirically observe how the feedback error behaves during:
- acceleration / cruise / deceleration,
- sudden target changes,
- changes in vmax and acceleration,
- manual braking and external disturbances,
- possible stalls / blocked motion detection.

Hardware:
- Servo signal: SERVO_PIN (default D9)
- Servo powered from external supply (recommended for torque servos)
- Common ground between Arduino and servo supply is mandatory
- User pots (0..5V dividers): A0, A1, A2
- Servo feedback wiper (after RC): A4

Recommended RC at Arduino input (already tested):
- 1 kΩ series from wiper -> Arduino ADC pin
- 4.7 µF from ADC pin -> GND (close to Arduino pin)

===============================================================================
*/

#include <Arduino.h>
#include <Servo.h>

// --------------------------- USER CONFIGURATION -----------------------------

#define SERVO_PIN        9          // PWM signal pin to servo
#define POT_TARGET_PIN   A0         // User pot: target angle command
#define POT_VMAX_PIN     A1         // User pot: max speed control
#define POT_ACCEL_PIN    A2         // User pot: acceleration control
#define POT_FB_PIN       A4         // Servo internal feedback (filtered) to A4

#define BAUDRATE         115200

#define VREF             5.0
#define ADC_SCALE        1023.0

#define LOOP_INTERVAL_MS 20         // Control tick (ms). 20 ms ~ servo frame time.
#define FB_SAMPLES       6          // Light averaging (RC already filters most noise)

// --------------------------- SERVO PWM CALIBRATION --------------------------
// These limits were measured for your HS-805BB example:
// center = 1500 us, endpoints at +/- 860 us for 0..180 degrees (real travel).
// Keep them configurable so you can reuse the sketch with other servos.

#define SERVO_CENTER_US  1500
#define SERVO_HALFSPAN_US 860       // +/- around center
#define PWM_MIN_US       (SERVO_CENTER_US - SERVO_HALFSPAN_US)   // 640 us
#define PWM_MAX_US       (SERVO_CENTER_US + SERVO_HALFSPAN_US)   // 2360 us

// --------------------------- FEEDBACK ADC CALIBRATION ------------------------
// Your measured servo internal potentiometer ADC values at real 0 and 180 degrees.
// You provided:
//   0 deg  -> ADC ≈  91
//   180 deg-> ADC ≈ 376
//
// Keep these configurable per servo.

static const float FB_ADC_AT_0_DEG   =  91.0f;
static const float FB_ADC_AT_180_DEG = 376.0f;

// Precompute degrees per ADC count (linear mapping)
static const float FB_DEG_PER_ADC =
  180.0f / (FB_ADC_AT_180_DEG - FB_ADC_AT_0_DEG);

// --------------------------- MOTION PROFILE RANGE ----------------------------
// These ranges map the user pots to useful units. Tune as needed.

#define VMAX_MIN_DEGPS   5.0f       // minimum max speed (deg/s)
#define VMAX_MAX_DEGPS   180.0f     // maximum max speed (deg/s)

#define ACCEL_MIN_DEGPS2 20.0f      // minimum accel (deg/s^2)
#define ACCEL_MAX_DEGPS2 1000.0f    // maximum accel (deg/s^2)

// --------------------------- GLOBALS ----------------------------------------

Servo s;

// Software profile state (the "commanded reference" we send to the servo)
float cmdDeg = 0.0f;               // current commanded angle (deg)
float vDegps = 0.0f;               // current profile velocity (deg/s)

// Timing
unsigned long lastTickMs = 0;

// --------------------------- SMALL HELPERS ----------------------------------

static inline float clampf(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

static inline float mapAdcToFloatRange(float adc, float outMin, float outMax) {
  float t = adc / ADC_SCALE;       // 0..1
  return outMin + t * (outMax - outMin);
}

// Averaged analog read (light smoothing without heavy CPU load)
float readAveragedADC(uint8_t pin, int samples) {
  long sum = 0;
  for (int i = 0; i < samples; i++) {
    sum += analogRead(pin);
  }
  return (float)sum / (float)samples;
}

// User target pot: ADC -> target degrees (0..180)
float targetDegFromPot(float adcTarget) {
  float deg = (adcTarget / ADC_SCALE) * 180.0f;
  return clampf(deg, 0.0f, 180.0f);
}

// Convert degrees -> PWM microseconds using calibrated endpoints
int pwmUsFromDeg(float deg) {
  deg = clampf(deg, 0.0f, 180.0f);
  float us = PWM_MIN_US + (deg / 180.0f) * (PWM_MAX_US - PWM_MIN_US);
  return (int)(us + 0.5f);
}

// Servo feedback ADC -> feedback degrees (0..180) using your calibration
float feedbackDegFromAdc(float adcFb) {
  float deg = (adcFb - FB_ADC_AT_0_DEG) * FB_DEG_PER_ADC;
  return clampf(deg, 0.0f, 180.0f);
}

// One trapezoidal-profile tick update:
// - uses vmax and accel limits
// - ensures we can stop by limiting velocity with v_stop = sqrt(2*a*d)
void updateProfile(float targetDeg, float vmaxDegps, float accelDegps2, float dt) {

  // Distance remaining (signed): positive means "need to move up"
  float dist = targetDeg - cmdDeg;
  float dir  = (dist >= 0.0f) ? 1.0f : -1.0f;
  float dabs = fabs(dist);

  // If we're essentially at the target, snap and zero velocity.
  if (dabs < 0.2f) {
    cmdDeg = targetDeg;
    vDegps = 0.0f;
    return;
  }

  // Compute maximum velocity that still allows stopping within remaining distance.
  // v_stop = sqrt(2 * a * d)
  float vStop = sqrtf(2.0f * accelDegps2 * dabs);

  // Effective allowed speed is the minimum of:
  // - user vmax
  // - stopping-limited speed
  float vAllowed = min(vmaxDegps, vStop);

  // Accelerate current velocity toward vAllowed, with accel limit.
  // We keep velocity always positive in magnitude, direction applied separately.
  float vMag = fabs(vDegps);
  vMag += accelDegps2 * dt;             // ramp up
  if (vMag > vAllowed) vMag = vAllowed; // limit by allowed speed

  // Apply direction
  vDegps = dir * vMag;

  // Integrate position
  float delta = vDegps * dt;

  // Prevent overshoot beyond target
  if (fabs(delta) > dabs) {
    cmdDeg = targetDeg;
    vDegps = 0.0f;
  } else {
    cmdDeg += delta;
  }
}

// ---------------------------------------------------------------------------
// SETUP
// ---------------------------------------------------------------------------

void setup() {
  Serial.begin(BAUDRATE);
  delay(500);

  // Attach servo with explicit pulse range (for predictable mapping in degrees)
  // Note: write() uses degrees internally mapped to attach(min,max) range.
  // We use writeMicroseconds() anyway, but attach(min,max) is still a good practice.
  s.attach(SERVO_PIN, PWM_MIN_US, PWM_MAX_US);

  // Read initial feedback angle so we start "where the servo is"
  float adcFb0 = readAveragedADC(POT_FB_PIN, FB_SAMPLES);
  float fbDeg0 = feedbackDegFromAdc(adcFb0);

  cmdDeg = fbDeg0;   // Initialize commanded reference to measured position
  vDegps = 0.0f;

  // Send a first PWM corresponding to this initial cmdDeg
  s.writeMicroseconds(pwmUsFromDeg(cmdDeg));

  lastTickMs = millis();

  Serial.println("servo_control_with_feedback");
  Serial.println("TargetDeg | CmdDeg | MeasDeg | ErrCmd-Meas | V_degps | A_degps2 | Vmax_degps | PWM_us | FB_ADC");
  Serial.println("----------------------------------------------------------------------------------------------------");
}

// ---------------------------------------------------------------------------
// LOOP
// ---------------------------------------------------------------------------

void loop() {

  unsigned long now = millis();
  if (now - lastTickMs < LOOP_INTERVAL_MS) {
    return; // Keep a stable update interval
  }

  // dt in seconds
  float dt = (now - lastTickMs) / 1000.0f;
  lastTickMs = now;

  // --- Read user controls (target, vmax, accel) ---
  float adcTarget = readAveragedADC(POT_TARGET_PIN, 4);  // light average is enough
  float adcVmax   = readAveragedADC(POT_VMAX_PIN,   4);
  float adcAccel  = readAveragedADC(POT_ACCEL_PIN,  4);

  float targetDeg = targetDegFromPot(adcTarget);

  float vmaxDegps = mapAdcToFloatRange(adcVmax,  VMAX_MIN_DEGPS,   VMAX_MAX_DEGPS);
  float accelDegps2 = mapAdcToFloatRange(adcAccel, ACCEL_MIN_DEGPS2, ACCEL_MAX_DEGPS2);

  // --- Update software motion profile (cmdDeg evolves smoothly) ---
  updateProfile(targetDeg, vmaxDegps, accelDegps2, dt);

  // --- Command servo with the current commanded reference ---
  int pwmUs = pwmUsFromDeg(cmdDeg);
  s.writeMicroseconds(pwmUs);

  // --- Read servo internal feedback (filtered) ---
  float adcFb = readAveragedADC(POT_FB_PIN, FB_SAMPLES);
  float measDeg = feedbackDegFromAdc(adcFb);

  // Error between what we are commanding *now* and what we measure *now*
  float errCmdMeas = cmdDeg - measDeg;

  // --- Print log line ---
  Serial.print(targetDeg, 1);     Serial.print(" | ");
  Serial.print(cmdDeg, 1);        Serial.print(" | ");
  Serial.print(measDeg, 1);       Serial.print(" | ");
  Serial.print(errCmdMeas, 1);    Serial.print(" | ");
  Serial.print(vDegps, 1);        Serial.print(" | ");
  Serial.print(accelDegps2, 0);   Serial.print(" | ");
  Serial.print(vmaxDegps, 1);     Serial.print(" | ");
  Serial.print(pwmUs);            Serial.print(" | ");
  Serial.println(adcFb, 2);
}
