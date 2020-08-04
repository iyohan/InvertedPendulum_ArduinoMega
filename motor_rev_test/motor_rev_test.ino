/* 20-02-15 John Lee
 * Arduino Project: Inverted Pendulum (One-Wheeled Balancing Cart)
 * Arduino Mega 2560
 */

#include <Arduino.h>
#include <Esplora.h>

void rhInterrupt();
void lhInterrupt();

// Pin Setting
const int RH_INA = 6;  // Right side EN
const int RH_INB = 7;  // Right side DIR
const int LH_INA = 8;  // Left side EN (speed)
const int LH_INB = 9;  // Left side DIR

const int RH_ENCA = 18;  // Right side encoder output A
const int RH_ENCB = 19;  // Right side encoder output B
const int LH_ENCA = 20;  // Left side encoder output A 
const int LH_ENCB = 21;  // Left side encoder output B

// Constants 
const int ENC_CHANGE_PER_REV = 32;  // Number of single channel rising/falling edges per revolution of motor shaft. 32 for a single channel.
const int GEAR_RATIO = 131.25;  // Gearbox ratio
const float RADIUS = 5.75;  // radius of wheel (cm)

const float INTERVAL_SENSOR = 10;
const float INTERVAL_CONTROL = 50; //ms
const float INTERVAL_PRINT = 1000;

// Variables
int rh_dir = 0;  // forward: 0, backward: 1
int rh_pwm = 128;  // 0~255
int lh_dir = 1 - rh_dir;
int lh_pwm = 128;

unsigned long time_curr = 0;
unsigned long sensor_time_prev = 0;
unsigned long control_time_prev = 0;
unsigned long print_time_prev = 0;

volatile long rh_encA_cnt = 0;
volatile long lh_encA_cnt = 0;

float rh_rev_motor = 0;
float rh_rev_wheel = 0;
float rh_dist_meas_curr = 0;
float rh_dist_meas_prev = 0;
float rh_speed_meas = 0;
int rh_dir_meas = 0;

float lh_rev_motor = 0;
float lh_rev_wheel = 0;
float lh_dist_meas_curr = 0;
float lh_dist_meas_prev = 0;
float lh_speed_meas = 0;
int lh_dir_meas = 0;


void setup() {
  pinMode(LH_INA, OUTPUT);
  pinMode(LH_INB, OUTPUT);
  pinMode(RH_INA, OUTPUT);
  pinMode(RH_INB, OUTPUT);
  
  pinMode(LH_ENCA, INPUT);
  pinMode(LH_ENCB, INPUT);
  pinMode(RH_ENCA, INPUT);
  pinMode(RH_ENCB, INPUT);
  
  attachInterrupt(digitalPinToInterrupt(LH_ENCA), lhInterrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RH_ENCA), rhInterrupt, CHANGE);
  
  Serial.begin(115200);
}


void loop() {

  time_curr = millis();

  // Sensor Reading
  if (time_curr - sensor_time_prev >= INTERVAL_SENSOR) {
    sensor_time_prev = time_curr;

    // Speed Calculation (Right-side)
    rh_rev_motor = rh_encA_cnt / ENC_CHANGE_PER_REV;
    rh_rev_wheel = rh_rev_motor / GEAR_RATIO;
    rh_dist_meas_curr = rh_rev_wheel * 2 * PI * RADIUS;
    rh_speed_meas = (rh_dist_meas_curr - rh_dist_meas_prev) / (INTERVAL_SENSOR / 1000); // Unit: cm/s

    rh_dist_meas_prev = rh_dist_meas_curr;
    //rh_encA_cnt = 0;

    // Speed Calculation (Left-side)
    lh_rev_motor = -lh_encA_cnt / ENC_CHANGE_PER_REV;
    lh_rev_wheel = lh_rev_motor / GEAR_RATIO;
    lh_dist_meas_curr = lh_rev_wheel * 2 * PI * RADIUS;
    lh_speed_meas = (lh_dist_meas_curr - lh_dist_meas_prev) / (INTERVAL_SENSOR / 1000); // Unit: cm/s

    lh_dist_meas_prev = lh_dist_meas_curr;
    //lh_encA_cnt = 0;

  }

  // Control
  if (time_curr - control_time_prev >= INTERVAL_CONTROL) {
    control_time_prev = time_curr;

    // PWM and DIR
    analogWrite(LH_INA, rh_pwm);
    analogWrite(RH_INA, lh_pwm);
    digitalWrite(LH_INB, rh_dir);
    digitalWrite(RH_INB, lh_dir);
  }

  // Serial I/O
  if(time_curr - print_time_prev >= INTERVAL_PRINT) {
    print_time_prev = time_curr;
    
    Serial.print("Rev_motor(R, L): ");
    Serial.print(rh_rev_motor);
    Serial.print(',');
    Serial.println(lh_rev_motor);
    
    if (Serial.available()) {
      String input = Serial.readString();
      int input_int = input.toInt();
      Serial.print("PWM value changed to ");
      Serial.println(input_int);
      Serial.print("\n\r");
      lh_pwm = input_int;
      rh_pwm = input_int;
    }
  }
}


void rhInterrupt()
{
  if (digitalRead(RH_ENCA) == HIGH) {
    if (digitalRead(RH_ENCB) == LOW) {
      rh_encA_cnt++;
    }
    else {
      rh_encA_cnt--;
    }
  }
  else {
    if (digitalRead(RH_ENCB) == LOW) {
      rh_encA_cnt--;
    }
    else {
      rh_encA_cnt++;
    }
  }
}


void lhInterrupt()
{
  if (digitalRead(LH_ENCA) == HIGH) {
    if (digitalRead(LH_ENCB) == LOW) {
      lh_encA_cnt++;
    }
    else {
      lh_encA_cnt--;
    }
  }
  else {
    if (digitalRead(LH_ENCB) == LOW) {
      lh_encA_cnt--;
    }
    else {
      lh_encA_cnt++;
    }
  }
}
