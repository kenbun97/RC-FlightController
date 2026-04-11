/**
 * Module Name: PlaneControl
 * Description: Controls the plane's servos and ESC.
 * Author: Ken Bunnell (Ported to C)
 * Date: 2026-04-08
 * License: GPL
 *
 * Notes:
 *  PWM Signal Min/Max Pulse Width:   990/2018us
 */

#include <Arduino.h>
#include "plane.h"

// Pin Definitions
#define THROTTLE_OUTPUT_PIN 13  // GPIO13
#define RUDDER_OUTPUT_PIN 14    // D14
#define ELEVATOR_OUTPUT_PIN 27  // D27

#define THROTTLE_INPUT_PIN 15   // D15
#define RUDDER_INPUT_PIN 21     // MISO
#define ELEVATOR_INPUT_PIN 32   // D32
#define MODE_INPUT_PIN 33       // D33

// PWM Properties for ESP32 LEDC
#define PWM_FREQ 50       // 50Hz for Servos/ESC
#define PWM_RES 10        // 10-bit resolution (0-1023)
#define ESC_CHAN 0
#define RUDDER_CHAN 1
#define ELEV_CHAN 2

#define PULSE_START 1
#define PULSE_END 0

#define SERVO_MIN       950

#define THROTTLE_MIN    1171
#define THROTTLE_MAX    2210
#define THROTTLE_SLOPE  6.44
#define THROTTLE_INT    1171
#define THROTTLE_INIT   585

#define RUDDER_MIN      1054
#define RUDDER_MAX      2040
#define RUDDER_SLOPE    12.32
#define RUDDER_INT      1547

#define ELEVATOR_MIN    1074
#define ELEVATOR_MAX    1894
#define ELEVATOR_SLOPE  -32.81
#define ELEVATOR_INT    1566

#define AUTO_MODE_THRESHOLD 1500

// Pre-calculates the scale to avoid division in the loop
// 1024 / 20000 simplified is 64 / 1250
#define US_TO_DUTY(us) ((us * 64) / 1250)

static volatile unsigned long throttleInpStart = 0;
volatile unsigned long throttleInpPWM = 0;
static volatile unsigned long rudderInpStart = 0;
volatile unsigned long rudderInpPWM = 0;
static volatile unsigned long elevatorInpStart = 0;
volatile unsigned long elevatorInpPWM = 0;
static volatile unsigned long modeInpStart = 0;
volatile unsigned long modeInpPWM = 0;
volatile bool autoMode = 0;

static void readPWM(const int pin, volatile unsigned long *startTime, volatile unsigned long *pwmVal);

// Convert PWM to Airfoil Deflection Angle
float pwmToThrottle(unsigned long pwm) {
  if(pwm < SERVO_MIN) {
    return NAN;
  }
  return (((long)pwm - THROTTLE_INT) / THROTTLE_SLOPE);
}

// Convert PWM to Airfoil Deflection Angle
float pwmToRudder(unsigned long pwm) {
  if(pwm < SERVO_MIN) {
    return NAN;
  }
  return (((long)pwm - RUDDER_INT) / RUDDER_SLOPE);
}

// Convert PWM to Airfoil Deflection Angle
float pwmToElevator(unsigned long pwm) {
  if(pwm < SERVO_MIN) {
    return NAN;
  }
  return (((long)pwm - ELEVATOR_INT) / ELEVATOR_SLOPE);
}

// Sets the ESC as percentage. 
// -1 will send a low signal to initialize the ESC.
// Note: duty values scaled from MicroPython 0-1023 range to 10-bit C range.
void setThrottle(int pulse) {
  if(pulse < 0) {
    pulse = THROTTLE_INIT;
  } else {
    pulse = constrain(pulse,THROTTLE_MIN,THROTTLE_MAX);
  }
  ledcWrite(ESC_CHAN, US_TO_DUTY(pulse));
}

// Sets angle of rudder between -40 and 40 deg.
void setRudder(int pulse) {
  pulse = constrain(pulse,RUDDER_MIN,RUDDER_MAX);
  ledcWrite(RUDDER_CHAN, US_TO_DUTY(pulse));
}

// Sets angle of elevator between -10 and 15 deg.
void setElevator(int pulse) {
  pulse = constrain(pulse,ELEVATOR_MIN,ELEVATOR_MAX);
  ledcWrite(ELEV_CHAN, US_TO_DUTY(pulse));
}

// Reads the pilot's throttle input
static void IRAM_ATTR pilotThrottleInput() {
  readPWM(THROTTLE_INPUT_PIN, &throttleInpStart, &throttleInpPWM);
}

// Reads the pilot's elevator input
static void IRAM_ATTR pilotElevatorInput() {
  readPWM(ELEVATOR_INPUT_PIN, &elevatorInpStart, &elevatorInpPWM);
}

// Reads the pilot's rudder input
static void IRAM_ATTR pilotRudderInput() {
  readPWM(RUDDER_INPUT_PIN, &rudderInpStart, &rudderInpPWM);
}

// Reads the pilot's auto/manual input
static void IRAM_ATTR pilotModeInput() {
  readPWM(MODE_INPUT_PIN, &modeInpStart, &modeInpPWM);
  autoMode = (modeInpPWM > AUTO_MODE_THRESHOLD);
}

// Reads the pulse length of PWM
// Called by interrupt each time the input changes in PWM cycle
static void readPWM(const int pin, volatile unsigned long *startTime, volatile unsigned long *pwmVal) {
    if (digitalRead(pin) == HIGH) {
        // Pulse started
        *startTime = micros();
    } else {
        // Pulse ended
        if (*startTime != 0) {
            *pwmVal = (int)(micros() - *startTime);
        }
    }
    return;
}

void debugInputPWM() {
  Serial.print(throttleInpPWM);
  Serial.print(" ");
  Serial.print(rudderInpPWM);
  Serial.print(" ");
  Serial.print(elevatorInpPWM);
  Serial.print(" ");
  Serial.println(modeInpPWM);
}

void setupPlane() {
    // Configure PWM Output Channels
    ledcAttach(THROTTLE_OUTPUT_PIN, PWM_FREQ, PWM_RES);
    ledcAttach(RUDDER_OUTPUT_PIN, PWM_FREQ, PWM_RES);
    ledcAttach(ELEVATOR_OUTPUT_PIN, PWM_FREQ, PWM_RES);

    // Default outputs to neutral
    setThrottle(-1);
    setElevator(0);
    setRudder(0);

    // Configure PWM Input Channels
    pinMode(THROTTLE_INPUT_PIN, INPUT_PULLDOWN);
    attachInterrupt(digitalPinToInterrupt(THROTTLE_INPUT_PIN), pilotThrottleInput, CHANGE);
    pinMode(RUDDER_INPUT_PIN, INPUT_PULLDOWN);
    attachInterrupt(digitalPinToInterrupt(RUDDER_INPUT_PIN), pilotRudderInput, CHANGE);
    pinMode(ELEVATOR_INPUT_PIN, INPUT_PULLDOWN);
    attachInterrupt(digitalPinToInterrupt(ELEVATOR_INPUT_PIN), pilotElevatorInput, CHANGE);
    pinMode(MODE_INPUT_PIN, INPUT_PULLDOWN);
    attachInterrupt(digitalPinToInterrupt(MODE_INPUT_PIN), pilotModeInput, CHANGE);
}