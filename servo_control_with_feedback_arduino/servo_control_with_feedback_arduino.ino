/*
===============================================================================
Title:        servo_control_with_feedback
Date:         2026-02-27
Author:       Alejandro Alonso Puig + ChatGPT
License:      Apache 2.0
-------------------------------------------------------------------------------
Description:
Closed-loop experimental servo control with trapezoidal motion profile and
real feedback measurement.

This version adds dynamic tracking diagnostics:
- Expected velocity (deg/s)
- Measured velocity (deg/s)
- Tracking ratio
- Simple stall detection logic
===============================================================================
*/

#include <Arduino.h>
#include <Servo.h>

// -------------------- USER CONFIGURATION --------------------

#define SERVO_PIN      9
#define POT_TARGET_PIN A0
#define POT_VMAX_PIN   A1
#define POT_ACC_PIN    A2
#define POT_FB_PIN     A4

#define LOOP_INTERVAL_MS  20
#define VREF              5.0
#define ADC_SCALE         1023.0

// HS-805BB calibrated limits
#define PWM_CENTER_US  1500
#define PWM_RANGE_US   860

#define PWM_MIN_US     (PWM_CENTER_US - PWM_RANGE_US)
#define PWM_MAX_US     (PWM_CENTER_US + PWM_RANGE_US)

// Feedback calibration (your measured values)
#define FB_ADC_0_DEG     91.0
#define FB_ADC_180_DEG   376.0

#define NUM_SAMPLES      4

// Stall detection thresholds
#define MIN_EXPECTED_VEL_FOR_TEST   10.0     // deg/s
#define MIN_REAL_VEL_THRESHOLD      2.0      // deg/s
#define STALL_CYCLES_REQUIRED       10

// ------------------------------------------------------------

Servo s;

float cmdDeg = 0.0;
float velocity = 0.0;

float prevMeasuredDeg = 0.0;

int stallCounter = 0;

// -------------------- HELPERS --------------------

float readADCavg(uint8_t pin)
{
  long sum = 0;
  for (int i = 0; i < NUM_SAMPLES; i++)
    sum += analogRead(pin);

  return (float)sum / NUM_SAMPLES;
}

float adcToDegCommand(float adc)
{
  return (adc / ADC_SCALE) * 180.0;
}

float adcToDegFeedback(float adc)
{
  float deg = (adc - FB_ADC_0_DEG) * 180.0 /
              (FB_ADC_180_DEG - FB_ADC_0_DEG);

  if (deg < 0) deg = 0;
  if (deg > 180) deg = 180;

  return deg;
}

int degToPWM(float deg)
{
  float us = PWM_MIN_US + (deg / 180.0) * (PWM_MAX_US - PWM_MIN_US);
  return (int)(us + 0.5);
}

// ------------------------------------------------------------

void setup()
{
  Serial.begin(115200);
  delay(1000);

  s.attach(SERVO_PIN, PWM_MIN_US, PWM_MAX_US);

  // Initialize from real measured position
  float adc_fb = readADCavg(POT_FB_PIN);
  cmdDeg = adcToDegFeedback(adc_fb);
  prevMeasuredDeg = cmdDeg;

  Serial.println("Target | Cmd | Meas | Err | Vexp | Vreal | Ratio | PWM");
  Serial.println("----------------------------------------------------------------");
}

// ------------------------------------------------------------

void loop()
{
  static unsigned long lastTime = millis();
  unsigned long now = millis();

  if (now - lastTime < LOOP_INTERVAL_MS)
    return;

  float dt = (now - lastTime) / 1000.0;
  lastTime = now;

  // ---- Read user inputs ----

  float targetDeg = adcToDegCommand(readADCavg(POT_TARGET_PIN));

  float vmax = (readADCavg(POT_VMAX_PIN) / ADC_SCALE) * 300.0;     // up to 300 deg/s
  float acc  = (readADCavg(POT_ACC_PIN) / ADC_SCALE) * 1000.0;     // up to 1000 deg/s²

  // ---- Trapezoidal profile ----

  float distance = targetDeg - cmdDeg;
  float dir = (distance >= 0) ? 1.0 : -1.0;
  float absDist = fabs(distance);

  float v_stop = sqrt(2.0 * acc * absDist);

  if (velocity > v_stop)
    velocity = v_stop;

  velocity += acc * dt;
  if (velocity > vmax)
    velocity = vmax;

  float delta = velocity * dt;

  if (delta > absDist)
    delta = absDist;

  cmdDeg += dir * delta;

  // ---- Send command ----

  int pwm = degToPWM(cmdDeg);
  s.writeMicroseconds(pwm);

  // ---- Measure real position ----

  float measuredDeg = adcToDegFeedback(readADCavg(POT_FB_PIN));

  float error = cmdDeg - measuredDeg;

  float velReal = (measuredDeg - prevMeasuredDeg) / dt;
  prevMeasuredDeg = measuredDeg;

  float velExpected = dir * velocity;

  float ratio = 0.0;
  if (fabs(velExpected) > 1.0)
    ratio = velReal / velExpected;

  // ---- Stall detection logic ----

  if (fabs(velExpected) > MIN_EXPECTED_VEL_FOR_TEST &&
      fabs(velReal) < MIN_REAL_VEL_THRESHOLD)
  {
    stallCounter++;
  }
  else
  {
    stallCounter = 0;
  }

  bool stalled = (stallCounter > STALL_CYCLES_REQUIRED);

  // ---- Print diagnostics ----

  Serial.print(targetDeg, 1); Serial.print(" | ");
  Serial.print(cmdDeg, 1);    Serial.print(" | ");
  Serial.print(measuredDeg, 1); Serial.print(" | ");
  Serial.print(error, 1);     Serial.print(" | ");
  Serial.print(velExpected, 1); Serial.print(" | ");
  Serial.print(velReal, 1);   Serial.print(" | ");
  Serial.print(ratio, 2);     Serial.print(" | ");
  Serial.print(pwm);

  if (stalled)
    Serial.print("  STALL");

  Serial.println();
}
