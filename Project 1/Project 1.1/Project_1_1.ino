#include "calibration2.h"   // CAL_A, CAL_B

const int trigPin = 13;     // Trigger pin for sensor 2
const int echoPin = 12;     // Echo pin for sensor 2
const int ledPin  = 9;      // LED

// Kalman filter parameters
const float Q_PROCESS = 0.0f;
const float R_MEAS    = 2.0f;
const float P_INIT    = 800.0f;

// Stop criterion (TIME-BASED)
const float VAR_THRESHOLD_TIME      = 25.0f;   
const float DELTA_TIME_THRESHOLD_US = 10.0f;    
const int   STABLE_COUNT_REQUIRED   = 10;

// KF state
float x_hat  = 0.0f;
float P      = P_INIT;
bool  initKF = false;

float prev_time_us = 0.0f;
int   stableCount  = 0;
bool  finished     = false;

// Time cost initialization
unsigned long Time1 = 0;
unsigned long Time2 = 0;

float readTimeUs();

void setup() {
  Serial.begin(9600);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(ledPin, OUTPUT);

  Serial.println("START");
  Time1 = millis();

  float z0 = readTimeUs();
  x_hat  = z0;
  P      = P_INIT;
  initKF = true;

  prev_time_us = x_hat;
}

void loop() {
  if (finished) return;

  // ---- Read raw time (us) ----
  float z = readTimeUs();

  // ---- Kalman filter on TIME ----
  float x_pred = x_hat;
  float P_pred = P + Q_PROCESS;

  float K = P_pred / (P_pred + R_MEAS);

  x_hat = x_pred + K * (z - x_pred);
  P     = (1.0f - K) * P_pred;

  float time_cost_us = x_hat;

  // Calibration
  float distance_mm  = CAL2_A * time_cost_us + CAL2_B;

  // Output
  Serial.print(distance_mm, 2);
  Serial.print(", ");
  Serial.println(P, 2);

  // ---- Stop criterion (TIME ONLY) ----
  float deltaTime = fabs(time_cost_us - prev_time_us);
  prev_time_us = time_cost_us;

  bool varSmall    = (P < VAR_THRESHOLD_TIME);
  bool changeSmall = (deltaTime < DELTA_TIME_THRESHOLD_US);

  if (varSmall && changeSmall) stableCount++;
  else stableCount = 0;

  if (stableCount >= STABLE_COUNT_REQUIRED) {
    finished = true;

    // Flash LED
    for (int i = 0; i < 3; i++) {
      digitalWrite(ledPin, HIGH); delay(100);
      digitalWrite(ledPin, LOW); delay(100);
    }


    // ---- Print total time_cost ----
    Time2 = millis();
    Serial.print(Time2 - Time1); Serial.print(" Milliseconds");

    return;
  }
}

// -------- Raw ultrasonic time (us) --------
float readTimeUs() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  unsigned long duration = pulseIn(echoPin, HIGH, 30000UL);

  if (duration == 0) {
    if (initKF) return x_hat;
    return 1500.0f;
  }

  return (float)duration;
}