#include "graph.h"
#include "PID.h"
#include "sensor.h"
#include "config.h"
#include "gyro.h"

int grid[GRID_SIZE][GRID_SIZE];//Declaration

float i_cord = 1.1;
float j_cord = 5.1;
int orientation = 2;

#define target_i 9
#define target_j 5

#define ENCA 3
#define ENCB 2
#define PWM_1 9
#define IN2 8
#define IN1 7
#define IN3 11
#define IN4 12
#define ENCC 5
#define ENCD 4
#define PWM_2 10
#define buffer 10
#define cell_length 20

volatile int posi_left = 0 ;    // Left encoder count
volatile int posi_right = 0;   // Right encoder count
volatile int avg_pos;
long prevT = 0;
float eprev = 0;
float eintegral = 0;
float tpmm = 62.5;//tpcm but lite

float target_forward = cell_length * tpmm;
// PID constants

// PID for target forward
float kp = 0.5;
float kd = 0.005;
float ki = 0.002;

// PID for centering with same motor speed
float kp_center = 0.8;
float ki_center = 0.0;
float kd_center = 0.05;

// PID for centering with sensor inputs 
float kp_center_sensor = 0.01;   
float ki_center_sensor = 0.0001;
float kd_center_sensor = 0.0005;      // 0.00015
float kq_center_sensor = 0.00001;

// Gyro
float ang_degrees = 0;

// Angular PID constants
const float ang_Kp = 2.5;
const float ang_Ki = 1;
const float ang_Kd = 0.5;

// Angular PID variables
float ang_previous_error = 0;
float ang_integral = 0;

//Declerations
void enqueue(int x, int y);
void dequeue();
void print_queue();
void flood_fill(int x, int y);
void initiate();
void reset() ;
int best_choice();

void forward(int choice);
void align_orientation(int direction);
void turn_right();
void turn_left();
void move_forward(float distance, int flag);
void caliberate() ;
void setMotor(int dir, int pwmVal, int pwm, int in1, int in2);
void readEncoder();
void celebrate();
void printGrid();

//Gyro 
void imu_setup();                         // Initializes the IMU sensor.
void control_motors(int directionA, int directionB, int speedA, int speedB);  // Controls motor direction and speed.
void turn(float target_angle);            // Executes a turn of a specified angle using PID control.
void angular_displacement(); 

float isRight();
float isLeft();
float isForward();

void celebrate()
{
  turn(90);
  turn(-90);
}

void setup()
{
    Serial.begin(9600);
    pinMode(16, INPUT_PULLUP);
    pinMode(17, INPUT_PULLUP);
    imu_setup();
    
    pinMode(ENCA,INPUT);
    pinMode(ENCB,INPUT);
    pinMode(ENCC,INPUT);
    pinMode(ENCD,INPUT);
    attachInterrupt(digitalPinToInterrupt(ENCA), readEncoder_left, RISING);
    attachInterrupt(digitalPinToInterrupt(ENCC), readEncoder_right, RISING);
    pinMode(PWM_1,OUTPUT);
    pinMode(IN1,OUTPUT);
    pinMode(IN2,OUTPUT);
    pinMode(PWM_2,OUTPUT);
    pinMode(IN3,OUTPUT);
    pinMode(IN4,OUTPUT);

    pinMode(A1,INPUT);
    pinMode(A2,INPUT);
    pinMode(A3,INPUT);
    pinMode(A4,INPUT);

//   Serial.println("Initiating");
//   initiate();
//   Serial.println("Reseting");
//   reset();
//   isForward();
//   isRight();
//   isLeft();
//   printGrid();
}

void loop ()
{
    initiate();
    Serial.print("Initiated!");

    move_forward(1,0);

    int choice;
    while (!((int) i_cord == target_i && (int) j_cord == target_j))
    {
        reset();
        isRight();
        isLeft();
        isForward();

        //orient();

        flood_fill(target_i, target_j);

        choice = best_choice();

        forward(choice);
        delay(100);
    }

    forward(4);

    celebrate();
}