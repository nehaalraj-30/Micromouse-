#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "graph.h"
#include "sensor.h"
#include "config.h"
#include <Arduino.h>

// External variables
extern int grid[GRID_SIZE][GRID_SIZE];
extern int orientation;
extern float i_cord; 
extern float j_cord;

// Motor PID control variables
extern volatile int posi_left;
extern volatile int posi_right;
extern volatile int avg_pos;
extern long prevT;
extern float eprev;
extern float eintegral;
extern float target_forward;
extern float new_target;
extern float kp;
extern float kd;
extern float ki;
extern float kp_center;
extern float kd_center;
extern float ki_center;
extern float kp_center_sensor;
extern float kd_center_sensor;
extern float ki_center_sensor;
extern float kq_center_sensor;

// Function declarations
void forward(int choice);
void align_orientation(int direction);
void turn_right();
void turn_left();
void move_forward(float distance, int flag);
void caliberate();
void setMotor(int dir, int pwmVal, int pwm, int in1, int in2);
void readEncoder_left();
void readEncoder_right();
void resetEncoder();

#endif