#include "calibration2.h"   // Coefficients for sensor 2
#include "calibration5.h"   // Coefficients for sensor 5

const int trig4 = 13;       // Trigger pin sensor 2
const int echo4 = 12;       // Echo pin sensor 2

const int trig5 = 11;       // Trigger pin sensor 5
const int echo5 = 10;       // Echo pin sensor 5

const int ledPin = 9;       // LED

// -------- Kalman parameters --------
// Q: process noise for distance
const float Q  = 0.0f;    // static target

// R1, R2: measurement noise variances
const float R1 = 2.0f;    // Sensor 2
const float R2 = 1.65f;   // Sensor 5

// initial uncertainty
const float P0 = 800.0f;  // Initial Variance

// -------- Stop thresholds --------
const float VAR_THRESHOLD_DIST = 25.0f;  
const float DELTA_DIST_THRESH  = 10.0f; 
const int   STABLE_N           = 10;

// -------- Kalman state --------
float y_hat  = 0.0f;
float P      = P0;
float prev_y = 0.0f;
int   stable = 0;
bool  done   = false;

// Time cost Initialization
unsigned long T0 = 0;
unsigned long T1 = 0;

// -------- Sensor read (µs) --------
float readTimeUs(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  unsigned long d = pulseIn(echoPin, HIGH, 30000UL);
  if (d == 0) return NAN;
  return (float)d;
}

void setup() {
  Serial.begin(9600);

  pinMode(trig4, OUTPUT);
  pinMode(echo4, INPUT);

  pinMode(trig5, OUTPUT);
  pinMode(echo5, INPUT);

  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);

  Serial.println("START");
  T0 = millis();

  // ---------- One-time initialization ----------
  float t4 = readTimeUs(trig4, echo4);
  delay(40);
  float t5 = readTimeUs(trig5, echo5);

  int retries = 0;
  while ((isnan(t4) || isnan(t5)) && retries < 5) {
    delay(30);
    t4 = readTimeUs(trig4, echo4);
    delay(30);
    t5 = readTimeUs(trig5, echo5);
    retries++;
  }

  if (!isnan(t4) && !isnan(t5)) {
    float z4 = CAL2_A * t4 + CAL2_B;
    float z5 = CAL5_A * t5 + CAL5_B;
    y_hat = 0.5f * (z4 + z5);   // Average of the two sensors
  } else {
    y_hat = 0.0f;               // Fallback
  }

  P = P0;
  prev_y = y_hat;
}

void loop() {
  if (done) return;

  // -------- Read sensors --------
  float t4 = readTimeUs(trig4, echo4);
  delay(40);
  float t5 = readTimeUs(trig5, echo5);

  if (isnan(t4) || isnan(t5)) return;
   
  // Calibration 
  float z1 = CAL2_A * t4 + CAL2_B;
  float z2 = CAL5_A * t5 + CAL5_B;

  // -------- Prediction --------
  float y_pred = y_hat;
  float P_pred = P + Q;

  // -------- Correction 1 (Sensor 2) --------
  float K1 = P_pred / (P_pred + R1);
  float y1 = y_pred + K1 * (z1 - y_pred);
  float P1 = (1.0f - K1) * P_pred;

  // -------- Correction 2 (Sensor 5) --------
  float K2 = P1 / (P1 + R2);
  y_hat = y1 + K2 * (z2 - y1);
  P     = (1.0f - K2) * P1;
  
  // Output
  Serial.print(y_hat, 2);
  Serial.print(", ");
  Serial.println(P, 2);

  // -------- Stop check --------
  float dy = fabs(y_hat - prev_y);
  prev_y = y_hat;
  if (P < VAR_THRESHOLD_DIST && dy < DELTA_DIST_THRESH) {
    stable++;
  } else {
    stable = 0;
  }

  if (stable >= STABLE_N) {
    done = true;

    // Blink LED
    for (int i = 0; i < 5; i++) {
      digitalWrite(ledPin, HIGH); delay(80);
      digitalWrite(ledPin, LOW);  delay(80);
    }


    // Time cost (ms)
    T1 = millis();
    Serial.print(T1 - T0); Serial.print(" Milliseconds");

    return;
  }
}
