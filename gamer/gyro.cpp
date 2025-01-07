#include "gyro.h"
#include "config.h"
#include "PID.h"
Thread angler;
String angle;

extern int orientation;

extern float ang_degrees;
extern const float ang_Kp;
extern const float ang_Ki;
extern const float ang_Kd;

extern float ang_previous_error;
extern float ang_integral;

unsigned long last_sampling_time = 0;


void imu_setup() {
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }
}


void control_motors(int directionA, int directionB, int speedA, int speedB) {
  speedA = constrain(speedA, 0, 255);
  speedB = constrain(speedB, 0, 255);

  digitalWrite(IN3, directionA > 0 ? HIGH : LOW);
  digitalWrite(IN4, directionA < 0 ? HIGH : LOW);
  
  digitalWrite(IN1, directionB > 0 ? HIGH : LOW);
  digitalWrite(IN2, directionB < 0 ? HIGH : LOW);

  analogWrite(my_PWM_1, speedA);
  analogWrite(my_PWM_2, speedB);
}



void angular_displacement() {
  float x, y, z;
  unsigned long current_time = millis();
  
  if (IMU.gyroscopeAvailable() && current_time - last_sampling_time >= (1000 / IMU.gyroscopeSampleRate())) {
    IMU.readGyroscope(x, y, z);  // Read gyroscope values

    // Update the angular displacement if significant rotation
    if (abs(z) > 1) {
      ang_degrees += (current_time - last_sampling_time) / 1000.0 * z;  // Update angle
    }
    last_sampling_time = current_time;
  }
}

// Non-blocking motor control loop with PID
void turn(float target_angle) {
  float sign = (target_angle < 0) ? -1 : 1;
  float abs_target = target_angle * sign;
  float current_angle = ang_degrees;
  float angular_target = current_angle * sign + abs_target;
  float ang_error;
  unsigned long last_time = millis();
  unsigned long current_time;
  float dt;

  ang_integral = 0;
  ang_previous_error = 0;

  while (true) {
    // Calculate time delta
    current_time = millis();
    dt = (current_time - last_time) / 1000.0;
    last_time = current_time;

    // Calculate PID error terms
    ang_error = angular_target - ang_degrees * sign;
    ang_integral += ang_error * dt;
    float ang_derivative = (ang_error - ang_previous_error) / dt;
    float output = ang_Kp * ang_error + ang_Ki * ang_integral + ang_Kd * ang_derivative;

    // Constrain the output to avoid overflow
    output = constrain(output, -100, 100);
    int direction = (output > 0) ? 1 : -1;
    direction *= sign;

    // Set motor speeds
    int speed = abs(output) > 30 ? abs(output) : 30;
    control_motors(-direction, direction, speed, speed);

    // Update the previous error for the next iteration
    ang_previous_error = ang_error;

    // Exit loop when target is reached
    if (abs(ang_error) < 3) {
      break;
    }

    // Non-blocking delay simulation for motor control
    angular_displacement();  // Continuously update the angular position
  }

  // Stop the motors and reset encoder
  control_motors(0, 0, 0, 0);
  resetEncoder();
}

void turn_right() {
  float distr = readSense430(forwardPin1, arr_siz);
  float distl = readSense430(forwardPin2, arr_siz);
  orientation = (orientation%4)+1;
  if (abs(distl-distr)>5 && (distl+distr)<20 )
  {
    turn(-45);
  }
  else
  {
  turn(-90);
  }
}

void turn_left() {
  orientation = (orientation+2)%4+1;
  float distr = readSense430(forwardPin1, arr_siz);
  float distl = readSense430(forwardPin2, arr_siz);
  if (abs(distl-distr)>5 && (distl+distr)<20 )
  {
    turn(+45);
  }
  else
  {
  turn(+90);
  }
}


// void angular_displacement() {
//   float x, y, z;
//   float sampling_time = 1.0 / IMU.gyroscopeSampleRate(); 

//   while (true) {
//     if (IMU.gyroscopeAvailable()) {
//       IMU.readGyroscope(x, y, z); 
//       if (abs(z) > 1) {
//         ang_degrees += sampling_time * z; 
//       }
//     }
//   }
// }

// void turn(float target_angle) {
//   angler.start(mbed::callback(angular_displacement));

//   int sign = 1;
//   if (target_angle < 0) { sign = -1; }
//   float abs_target = target_angle * sign;
//   float current_angle = ang_degrees;
//   float angular_target = current_angle * sign + abs_target;
//   float ang_error;
//   float last_time = millis();
//   float dt;

//   ang_integral = 0;
//   ang_previous_error = 0;

//   do {
//     float current_time = millis();
//     dt = (current_time - last_time) / 1000.0;
//     last_time = current_time;

//     ang_error = angular_target - ang_degrees * sign;
    
//     ang_integral += ang_error * dt;
//     float ang_derivative = (ang_error - ang_previous_error) / dt;
//     float output = ang_Kp * ang_error + ang_Ki * ang_integral + ang_Kd * ang_derivative;

//     output = constrain(output, -100, 100);
//     int direction = (output > 0) ? 1 : -1;
//     direction *= sign;

//     int speed = 30;
//     if (output > 30) {
//       speed = abs(output);
//     }

//     control_motors(-direction, direction, speed, speed);

//     ang_previous_error = ang_error;
//     //Serial.println(ang_degrees);

//     delay(10);
//   } while (abs(ang_error) >= 10);

//   resetEncoder();
//   control_motors(0, 0, 0, 0);

//   angler.terminate();
// }