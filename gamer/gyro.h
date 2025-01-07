#ifndef MICROMOUSE_H
#define MICROMOUSE_H

#include "Arduino_BMI270_BMM150.h"
#include <ArduinoBLE.h>
#include "mbed.h"
#include <rtos.h>

using namespace rtos;

extern Thread angler;

extern String angle;
extern bool connected;

// Angular PID constants
extern const float ang_Kp;
extern const float ang_Ki;
extern const float ang_Kd;

// Angular PID variables
extern float ang_previous_error;
extern float ang_integral;
extern float ang_degrees;

void imu_setup();
void motor_setup();
void control_motors(int directionA, int directionB, int speedA, int speedB);
void turn(float target_angle);
void angular_displacement();
void turn_right();
void turn_left();

#endif
