# servo_control_with_feedback

This repository contains `servo_control_with_feedback`, an Arduino sketch designed to experimentally study servo motion shaping **with real position feedback** from the internal potentiometer of the servo.

Unlike the previous open-loop version, this sketch reads the servo’s internal feedback signal (externally accessed and filtered) and logs the measured angle together with the commanded reference. The goal is to empirically analyze dynamic error behavior under different motion conditions.

## Objective

The sketch generates a trapezoidal motion profile in software, defined by:

- Target angle (degrees)  
- Maximum velocity `v_max (deg/s)`  
- Acceleration `a (deg/s²)`  

At the same time, it measures the actual servo position using the internal potentiometer (via `A4`) and computes:

Error = commanded_position − measured_position  

The commanded position refers to the *instantaneous reference value* produced by the motion profile, not the final target angle.

This allows observation of error behavior during:

- Acceleration phase  
- Constant velocity cruise  
- Deceleration phase  
- Sudden target changes  
- Changes in velocity and acceleration  
- Manual braking or mechanical blocking  
- External disturbances  

The purpose is to evaluate whether the feedback signal can be used to detect abnormal behavior such as friction increase, overload, or mechanical collision.

## Hardware setup

An Arduino Uno (or compatible 5 V board), one standard RC servo, and three user potentiometers are required.

Connections:

- Servo signal → `D9` (configurable as `SERVO_PIN`)  
- Servo power → external 6 V supply recommended for high-torque servos  
- Servo ground → supply ground  
- Arduino ground → must be connected to the same ground as the servo supply  

User potentiometers (voltage dividers 0–5 V):

- Target angle → `A0`  
- Maximum velocity → `A1`  
- Acceleration → `A2`  

Servo internal feedback:

- Servo potentiometer wiper → `A4`  
- Recommended analog filter at Arduino input:  
  - 1 kΩ series resistor from wiper to `A4`  
  - 4.7 µF capacitor from `A4` to GND  

The RC filter reduces burst noise and PWM-induced interference from the servo electronics.

High-torque servos must not be powered from the Arduino 5 V rail.

## Control principle

At a fixed interval (`LOOP_INTERVAL_MS`), the sketch:

1. Reads the target angle from `A0` and maps it to 0–180°.  
2. Reads `v_max` and acceleration from `A1` and `A2`.  
3. Updates an internal commanded reference (`cmdDeg`) using a trapezoidal motion profile.  
4. Converts the commanded angle to calibrated PWM microseconds.  
5. Sends the PWM signal to the servo.  
6. Reads the internal feedback angle from `A4`.  
7. Computes instantaneous error between commanded and measured position.  

The trapezoidal profile uses:

- `v = v + a·dt` for acceleration  
- Velocity limiting by user-defined `v_max`  
- Stopping constraint `v_stop = sqrt(2·a·distance)`  

This ensures smooth acceleration and deceleration of the *reference signal*.

## Startup behavior

At power-up, the sketch reads the feedback potentiometer and initializes the commanded reference to the measured angle. This prevents any startup jump and ensures the system begins from the actual physical position of the servo.

## Servo calibration

For the tested HS-805BB servo, real mechanical endpoints were measured as approximately:

- 0° → 1500 − 860 µs  
- 180° → 1500 + 860 µs  

Feedback ADC calibration values:

- 0° → ADC ≈ 91  
- 180° → ADC ≈ 376  

Both PWM limits and ADC calibration constants are configurable in the sketch and must be adapted if another servo is used.

## Serial output

At each control update (115200 baud via USB), the sketch prints:

- Target angle (deg)  
- Commanded angle (deg)  
- Measured angle (deg)  
- Instantaneous error (deg)  
- Current profile velocity (deg/s)  
- Acceleration (deg/s²)  
- Maximum velocity (deg/s)  
- PWM command (µs)  
- Raw feedback ADC value  

This enables detailed time-series analysis of dynamic error behavior.

## Interpretation of error behavior

RC servos contain an internal closed-loop controller, typically proportional or PD-based. Motor drive effort is proportional to position error between the commanded pulse width and the internal feedback potentiometer.

Because this sketch incrementally updates the commanded reference:

- Large reference jumps create large internal error and maximum motor effort.  
- Small incremental updates keep error small and reduce internal drive effort.  
- During acceleration, error tends to increase.  
- During cruise, error stabilizes.  
- During deceleration, error reduces.  
- Under mechanical blocking, error grows and remains sustained.  

By observing the magnitude and duration of the error, it becomes possible to empirically evaluate whether position feedback could be used for collision or stall detection in higher-level control systems.
