#include "sensor.h"
#include "config.h"

// Define the global variables (to be shared with other files)
extern int grid[GRID_SIZE][GRID_SIZE];
extern int orientation;
extern float i_cord;
extern float j_cord;

// These are sensor functions
float volt;
float Distance;

// Function to sort an array in descending order
int sort_desc(const void *cmp1, const void *cmp2) {
    int a = *((int *)cmp1);
    int b = *((int *)cmp2);
    return a > b ? -1 : (a < b ? 1 : 0);
}

// Function to read sensor data (215 model)
float readSense215(int Apin, int arr_length) {
    float sense[arr_length];
    for (int i = 0; i < 5;) {
        volt = analogRead(Apin) * (0.0048828125);  // Convert to voltage
        Distance = 5.2819 * pow(volt, -1.161);     // Convert to distance
        if (1) {
            sense[i] = Distance;
            i++;
        }
        delay(10);  // Debounce delay
    }
    qsort(sense, arr_length, sizeof(sense[0]), sort_desc);
    return sense[2];  // Return the median value
}

// Function to read sensor data (430 model)
float readSense430(int Apin, int arr_length) {
    float sense[arr_length];
    for (int i = 0; i < 5;) {
        volt = analogRead(Apin) * (0.0048828125);  // Convert to voltage
        Distance = 12.08 * pow(volt, -1.058);      // Convert to distance
        if (1) {
            sense[i] = Distance;
            i++;
        }
        delay(10);  // Debounce delay
    }
    qsort(sense, arr_length, sizeof(sense[0]), sort_desc);
    return sense[2];  // Return the median value
}

// Function to detect if there is an obstacle to the right
float isRight() {
    int cell_i = (int)i_cord;
    int cell_j = (int)j_cord;

    float dist = readSense215(rightPin, arr_siz);

    if (dist < 30) {  // Mark grid as obstacle if close enough
        switch (orientation) {
        case 1: grid[cell_i + 1][cell_j] = 69; break;
        case 2: grid[cell_i][cell_j - 1] = 69; break;
        case 3: grid[cell_i - 1][cell_j] = 69; break;
        case 4: grid[cell_i][cell_j + 1] = 69; break;
        }
    }

    return dist;
}

// Function to detect if there is an obstacle to the left
float isLeft() {
    int cell_i = (int)i_cord;
    int cell_j = (int)j_cord;

    float dist = readSense215(leftPin, arr_siz);

    if (dist < 30) {  // Mark grid as obstacle if close enough
        switch (orientation) {
        case 1: grid[cell_i - 1][cell_j] = 69; break;
        case 2: grid[cell_i][cell_j + 1] = 69; break;
        case 3: grid[cell_i + 1][cell_j] = 69; break;
        case 4: grid[cell_i][cell_j - 1] = 69; break;
        }
    }

    return dist;
}

// Function to detect if there is an obstacle in front
float isForward() {
    int cell_i = (int)i_cord;
    int cell_j = (int)j_cord;

    float dist1 = readSense430(forwardPin1, arr_siz);
    float dist2 = readSense430(forwardPin2, arr_siz);
    float dist = (dist1 + dist2) / 2;

    if (dist < 30) {  // Mark grid as obstacle if close enough
        switch (orientation) {
        case 1: grid[cell_i][cell_j + 1] = 69; break;
        case 2: grid[cell_i + 1][cell_j] = 69; break;
        case 3: grid[cell_i][cell_j - 1] = 69; break;
        case 4: grid[cell_i - 1][cell_j] = 69; break;
        }
    }

    return dist;
}