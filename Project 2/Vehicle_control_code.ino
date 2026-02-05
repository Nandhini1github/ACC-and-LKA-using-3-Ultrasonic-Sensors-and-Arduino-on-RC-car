// ========================
// Servo Library and Set-up
// ========================
#include <Servo.h> //define the servo library
Servo ssm; //create an instance of the servo object called ssm
Servo esc; //create an instance of the servo object called esc
int steering=1500, throttle=1500; //defining global variables to use later

// Including the calibration coefficients
#include "calibration2.h"   // for sensor 2
#include "calibration5.h"   // for sensor 5
#include "calibration3.h"   // for sensor 3

// =======================================
// Speed Limit (Throttle) and Sensor Limit
// =======================================
const int MAX_SPEED_US = 1620;   // upper speed limit
const int MIN_SPEED_US = 1390;   // allowing for reverse
const int MAX_VALID_DIST = 700; // mm
int velocity_us;
const int FAILSAFE_SPEED_US = 1640; //failsafe speed when sensor fails


// =============================
// Ultrasonic Sensor Pins set-up
// =============================
#define trigPinF1 13     // First Ultrasonic trigger pin is at pin 13 SENSOR 2
#define echoPinF1 12     // First Ultrasonic echo pin is at pin 12 SENSOR 2
#define trigPinF2 11     // Second Ultrasonis trigger pin is at pin 11 SENSOR 5
#define echoPinF2 10     // Second Ultrasonic echo pin is at pin 10 SENSOR 5
#define trigPinR 8       // Right side Ultrasonic trigger pin is at pin 8 SENSOR 3
#define echoPinR 7       // Right side Ultrasonic echo pin is at pin 7 SENSOR 3

// ================
// Desired behavior
// ================
const float DESIRED_DIST = 360.0;    // mm
const float DESIRED_SIDE = 315.0;    // mm
const float SAMPLE_TIME = 0.05;     // sec
const float Stop1 = DESIRED_DIST - 30; // mm within 33 cm from sensor
const float Stop2 = DESIRED_DIST + 50; // mm within 41 cm from sensor

// ================================
// Reading the raw signals function
// ================================
float readTimeResponse(float trigPin, float echoPin){
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  long time = pulseIn(echoPin, HIGH);
  return (float)time;
}

// ==============================
// PID Gains for Steering Control
// ==============================
float Ki_steer = 3;             //Integral gain (steering)
float Kd_steer = 0.9;             //Derivative gain (steering)
float Kp_steer = 4.1;             //Proportional gain (steering)

// ==============================
// PID Gains for Throttle Control
// ==============================
float Ki_drive = 3;             //Integral gain (throttle)
float Kd_drive = 17;             //Derivative gain (throttle)
float Kp_drive = 1;             //Proportional gain (throttle)

// ==========
// PID states
// ==========
float prevPrevDistError = 0; float prevDistError = 0; float distError = 0;
float prevPrevLaneError = 0; float prevLaneError = 0; float laneError = 0;

// ============
// PID Function
// ============
float PID(float actual, float desired, float &prevPrevError, float &prevError, float &error,
          float Kp, float Ki, float Kd)
{
    float K1 = Kp + Ki + Kd;
    float K2 = -Kp - 2.0f*Kd;
    float K3 = Kd;
    prevPrevError = prevError;
    prevError = error;
    error = actual - desired;
    return K1*error + K2*prevError + K3*prevPrevError;
}

void setup() {
   Serial.begin(9600); //start serial connection. Uncomment for PC
   ssm.attach(6); //define that ssm is connected at pin 6
   esc.attach(5); //define that esc is connected at pin 5
   pinMode(trigPinF1, OUTPUT);   //Making the trigger pin an output signal (front 1)
   pinMode(echoPinF1, INPUT);    //Making the echo pin an input signal (front 1)
   pinMode(trigPinF2, OUTPUT);   //Making the trigger pin an output signal (front 2)
   pinMode(echoPinF2, INPUT);    //Making the echo pin an input signal (front 2)
   pinMode(trigPinR, OUTPUT);    //Making the trigger pin an output signal (right)
   pinMode(echoPinR, INPUT);     //Making the echo pin an input signal (right)
   ssm.writeMicroseconds(1500);  //write steering value to steering servo
   esc.writeMicroseconds(1500);  //write velocity value to the ESC unit
   delay(1000); // Allow ESC to arm
}

void loop() {
  delay((int)(SAMPLE_TIME*1000));

  // =============================================================
  // Reading and calculating the distance of the front two sensors
  // =============================================================
  //1. Reading the raw data
  float Front1_raw_data = readTimeResponse(trigPinF1, echoPinF1);
  float Front2_raw_data = readTimeResponse(trigPinF2, echoPinF2);

  //2. Calibrating the data for the two sensors
  float distance_F1 = CAL2_A * Front1_raw_data + CAL2_B;
  float distance_F2 = CAL5_A * Front2_raw_data + CAL5_B;

  float Front_dist = ((distance_F1 + distance_F2)/2.0f);

  // ==================================================
  // Reading and calculating the right sensor
  // ==================================================
  //1. Reading the raw data
  float Right_raw_data = readTimeResponse(trigPinR, echoPinR);


  //2. Calibrating the two data sets to get the distance
  float dist_Right = (CAL3_A * Right_raw_data + CAL3_B);


  // ===================================================
  // Operating the PID looping for throttle and steering
  // ===================================================
  // Creating sample time index
  float dt = SAMPLE_TIME;

  //1. Lane Keeping PID
  float steerCMD = PID(dist_Right, DESIRED_SIDE, prevPrevLaneError, prevLaneError, laneError, Kp_steer, Ki_steer, Kd_steer);

  // mapping the output into servo microseconds
  int steering_us = 1500 + steerCMD;  //1500 = straight

  steering_us = constrain(steering_us, 1350, 1650);

  //2. Adaptive Cruise Control PID and constant velocity if sensor limit is reached
  bool sensorValid(Front_dist < MAX_VALID_DIST);
  bool StopCriterion(Front_dist > Stop1 && Front_dist < Stop2);
  
  if(sensorValid){
    if(StopCriterion){
      // *** BRAKING CRITERION ***
      // Setting the braking criterion if within threshold
      velocity_us = 1500;
      
      // Reset PID states so it doesn't spike when readings return
      prevPrevDistError = 0;
      prevDistError = 0;
      distError = 0;
    }

    else{
      //Applying the PID Looping
      float velCMD = PID(Front_dist, DESIRED_DIST, prevPrevDistError, prevDistError, distError, Kp_drive, Ki_drive, Kd_drive);

      velocity_us = 1500 + velCMD; //1500 = neutral

    // ======================
    // Apply throttle limits
    // ======================
    velocity_us = constrain(velocity_us, MIN_SPEED_US, MAX_SPEED_US);
    }
  }

  else{
    // *** FAILSAFE MODE ***
    // Use constant speed when the front sensor is unreliable
    velocity_us = FAILSAFE_SPEED_US;

    // Reset PID states so it doesn't spike when readings return
    prevPrevDistError = 0;
    prevDistError = 0;
    distError = 0;
    }

  //3. Apply to the vehicle
  setVehicle(steering_us, velocity_us); //steering_us

  Serial.print("Front 1 = "); Serial.print(distance_F1);
  Serial.print(" |Front 2 = "); Serial.print(distance_F2);
  Serial.print(" |Front = "); Serial.print(Front_dist);
  Serial.print(" | Side = "); Serial.print(dist_Right);
  Serial.print(" | LaneErr = "); Serial.print(laneError);
  Serial.print(" | SteerUS = "); Serial.print(steering_us);
  Serial.print(" | VelUS = "); Serial.println(velocity_us);
}

//********************** Vehicle Control **********************//
//***************** Do not change below part *****************//

void setVehicle(int s, int v)
{
  s=min(max(1050,s),1950); //saturate steering command
  v=min(max(1250,v),1750); //saturate velocity command
  ssm.writeMicroseconds(s); //write steering value to steering servo
  esc.writeMicroseconds(v); //write velocity value to the ESC unit
}
//***************** Do not change above part *****************//
