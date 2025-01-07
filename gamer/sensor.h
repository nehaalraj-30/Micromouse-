#ifndef SENSOR_H
#define SENSOR_H

#include <Arduino.h>
#include "config.h"

// External variables (from main program or other files)
extern int grid[GRID_SIZE][GRID_SIZE];
extern int orientation;
extern float i_cord; 
extern float j_cord;

// Function declarations
int sort_desc(const void* cmp1, const void* cmp2);
float readSense215(int Apin, int arr_size);
float readSense430(int Apin, int arr_size);
float isRight();
float isLeft();
float isForward();

#endif