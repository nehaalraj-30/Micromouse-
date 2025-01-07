#include "PID.h"
#include "config.h"

// Encoder position variables
extern volatile int posi_left;    // Left encoder count
extern volatile int posi_right;   // Right encoder count

// Initialize PID constants and variables
extern long prevT;
extern float eprev;
extern float eintegral;
extern float target_forward;
//extern float new_target;
extern float kp;
extern float kd;
extern float ki;

extern volatile int avg_pos;

extern int orientation;

// PID variables for centering
float e_center_prev = 0.0;
float e_center_integral ;

// PID constants for centering (tune these values)
extern float kp_center ;
extern float ki_center ;
extern float kd_center ;

float e_center_prev_sensor = 0.0;
float e_center_integral_sensor = 0.0;

extern float kp_center_sensor ;
extern float ki_center_sensor ;
extern float kd_center_sensor ;
extern float kq_center_sensor ;

// Control Loop Parameters     
const float center_buffer = 2.0;   

const int cell_width = 20;

extern float tpmm;

// Function Definitions

void forward(int choice) {
    align_orientation(choice);
    move_forward();
}

void align_orientation(int direction) {
    while (direction != orientation) {
        if ((direction - orientation + 4) % 4 == 1) {
            turn_right();
        } else {
            turn_left();
        }
    }
}

void move_forward() {
    // Reset encoders before starting the movement
    resetEncoder();  // Resets posi_left and posi_right to 0

    float e_forward = 0.0;    // Forward error
    float dedt_forward = 0.0; // Derivative of forward error
    float u_forward = 0.0;    // Control output for forward movement

    float e_center = 0.0;     // Centering error
    float dedt_center = 0.0;  // Derivative of centering error
    float u_center = 0.0;     // Control output for centering

    float e_center_sensor = 0.0;     // Centering error from sensors
    float dedt_center_sensor = 0.0;  // Derivative of centering error
    float u_center_sensor = 0.0;     // Control output for sensor centering

    while (true) {
        // Time difference calculation
        long currT = micros();
        float deltaT = ((float)(currT - prevT)) / 1.0e6; 
        prevT = currT;

        // Sensor readings
        float sensor_left = isLeft();
        float sensor_right = isRight(); 
        
        // Read encoder values and calculate average position
        int pos_left;
        int pos_right;
        noInterrupts(); // Temporarily disable interrupts to safely read values
        pos_left = posi_left;
        pos_right = posi_right;
        avg_pos = -(pos_left + pos_right) / 2;
        interrupts();    // Re-enable interrupts after reading
       
        // Calculate forward error (difference between target and current average position)
        e_forward = avg_pos - target_forward;

        // Break if within the buffer range
        if (abs(e_forward) <= buffer_siz) {
            break;
        }

        // Derivative and integral of forward error
        dedt_forward = (e_forward - eprev) / deltaT;
        eintegral += e_forward * deltaT;

        // PID control signal for forward movement
        u_forward = kp * e_forward + kd * dedt_forward + ki * eintegral;

        int min = (sensor_left>sensor_right) ? sensor_right : sensor_left;

        if(min<20 && 4>min || 6<min)
        {
            // Centering with sensors: Calculate centering error based on sensor values
            if (sensor_left > cell_width && sensor_right > cell_width) { // no wall both sides
                e_center_sensor = 0;
            } else if (sensor_left > cell_width) { // Wall on the right
                e_center_sensor = 2*(cell_width / 2 - sensor_right) * tpmm;
            } else if (sensor_right > cell_width) { // Wall on the left
                e_center_sensor = -2*(cell_width / 2 - sensor_left) * tpmm; // Negative sign is important
            } else {
                e_center_sensor = sensor_left * tpmm - sensor_right * tpmm;
            }

            // Derivative and integral of sensor-based centering error
            dedt_center_sensor = (e_center_sensor - e_center_prev_sensor) / deltaT;
            e_center_integral_sensor += e_center_sensor * deltaT;

            // Compute PID control signal for sensor-based centering
            u_center_sensor = kq_center_sensor * e_center_sensor * abs(e_center_sensor) + kp_center_sensor * e_center_sensor + kd_center_sensor * dedt_center_sensor + ki_center_sensor * e_center_integral_sensor;
        }

        if (min>20.0 || 4.5<min && 5.5>min)
        {
            // Calculate centering error (difference between left and right encoder counts)
            e_center = -(pos_left - pos_right);  // Desired: e_center = 0 (centered), retarded - sign

            // Derivative and integral of centering error
            dedt_center = (e_center - e_center_prev) / deltaT;
            e_center_integral += e_center * deltaT;

            // Compute PID control signal for centering (without sensor)
            u_center = kp_center * e_center + kd_center * dedt_center + ki_center * e_center_integral;
        }

        // Base speed for forward movement
        float base_speed = abs(u_forward); // Ensure positive speed
        base_speed = constrain(base_speed, 0, 105); // Constrain speed within valid PWM range (0-255)

        // Calculate steering adjustments
        float steering_sensor = u_center_sensor;
        float steering = u_center; // Positive: steer left, Negative: steer right

        // Define maximum steering adjustment
        float max_steer = 50; // Adjust as necessary
        steering = constrain(steering, -max_steer, max_steer);
        steering_sensor = constrain(steering_sensor, -max_steer, max_steer);

        // Compute final motor speeds
        float left_pwr = base_speed - steering_sensor - steering;
        float right_pwr = base_speed + steering_sensor + steering;

        // Constrain motor power within the valid range (0-255)
        left_pwr = constrain(abs(left_pwr), 0, 255);
        right_pwr = constrain(abs(right_pwr), 0, 255);

        // Ensure minimum motor power (prevent motors from stalling)
        if (left_pwr < 30) { left_pwr = 30; }
        if (right_pwr < 30) { right_pwr = 30; }

        // Determine motor directions based on forward control output
        int left_dir = (u_forward >= 0) ? 1 : -1;
        int right_dir = (u_forward >= 0) ? 1 : -1;

        // Set motor speeds and directions
        setMotor(left_dir, left_pwr, my_PWM_1, IN1, IN2);   // Left motor
        setMotor(right_dir, right_pwr, my_PWM_2, IN3, IN4); // Right motor

        // Update previous errors for the next iteration
        eprev = e_forward;
        e_center_prev = e_center;
        e_center_prev_sensor = e_center_sensor;

      
    }

    switch (orientation) {//TBD
            case 1:
                j_cord +=2;//+= avg_pos/tpmm;
                break;
            case 2:
                i_cord +=2;//+= avg_pos/tpmm;
                break;
            case 3:
                j_cord -=2;//+= -avg_pos/tpmm;
                break;
            case 4:
                i_cord -=2;//+= -avg_pos/tpmm;
                break;
        }
    
    // Stop both motors after completing the movement
    setMotor(0, 0, my_PWM_1, IN1, IN2);
    setMotor(0, 0, my_PWM_2, IN3, IN4);
}





void readEncoder_left() {
    int b = digitalRead(ENCB);
    if (b > 0) {
        posi_left--;
    } else {
        posi_left++;
    }
      
}

void readEncoder_right() {
    int d = digitalRead(ENCD);
    if (d > 0) {
        posi_right++;
    } else {
        posi_right--;
    }
    
}

void setMotor(int dir, int pwmVal, int pwm, int in1, int in2) {
    analogWrite(pwm, pwmVal);
    if (dir == 1) {
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);
    }
    else if (dir == -1) {
        digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);
    } else {
        digitalWrite(in1, LOW);
        digitalWrite(in2, LOW);
    }
}

void resetEncoder()
{
  noInterrupts();
  posi_left = 0;
  posi_right = 0;
  interrupts();
}

void caliberate() {
    int target = 0;
    int factor = 250;
    float deltaT = 0;
    int e = 0;
    float u = 0;
    float dedt = 0;

    int initial = isForward();

    while (target < factor) {
        target = factor * (1 / (pow(prevT / 1e6, 6) + 1));

        long currT = micros();
        deltaT = ((float)(currT - prevT)) / 1.0e6;
        prevT = currT;

        // Read the position
        int pos = 0;
        noInterrupts(); // Disable interrupts temporarily while reading
        pos = posi;
        interrupts(); // Turn interrupts back on

        // Error
        e = pos - target_forward;

        // Derivative
        dedt = (e - eprev) / deltaT;

        // Integral
        eintegral += e * deltaT;

        // Control signal
        u = kp * e + kd * dedt + ki * eintegral;

        // Motor power
        float pwr = fabs(u);
        if (pwr > 255) {
            pwr = 255;
        }

        // Motor direction
        int dir = 1;
        if (u < 0) {
            dir = -1;
        }

        // Signal the motor
        setMotor(dir, pwr, my_PWM_1, IN1, IN2);
        setMotor(dir, pwr, my_PWM_2, IN3, IN4);

        // Store previous error
        eprev = e;
    }

    int final = isForward();
    
    float tpmm = posi / (initial - final);
}

