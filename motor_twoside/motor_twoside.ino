/* 20-02-11 John Lee
 * Arduino Project: Inverted Pendulum
 */

void lhInterrupt();
void rhInterrupt();

// Pin Setting
const int LH_INA = 6;  // Left side EN (speed)
const int LH_INB = 7;  // Left side DIR
const int RH_INA = 8;  // Right side EN
const int RH_INB = 9;  // Right side DIR

const int LH_ENCA = 2;  // Left side encoder output A 
const int LH_ENCB = 3;  // Left side encoder output B
const int RH_ENCA = 4;  // Right side encoder output A
const int RH_ENCB = 5;  // Right side encoder output B

// Constants 
const int ENC_CHANGE_PER_REV = 16;  // Number of single channel rising/falling edges per revolution of motor shaft
const int GEAR_RATIO = 131.25;  // Gearbox ratio
const float RADIUS = 5.75;  // radius of wheel (cm)

const float INTERVAL_CONTROL = 100; //ms
const float INTERVAL_PRINT = 1000;

// Variables
int lh_dir = 0;  // forward: 0, backward: 1
int lh_pwm = 0;  // 0~255
int rh_dir = 0;
int rh_pwm = 0;

unsigned long time_curr = 0;
unsigned long time_prev = 0;
unsigned long print_time_prev = 0;

volatile long lh_encA_cnt = 0;
volatile long rh_encA_cnt = 0;

float rev_motor = 0;
float rev_wheel = 0;
float dist_meas_curr = 0;
float dist_meas_prev = 0;
float speed_meas = 0;
int dir_meas = 0;


void setup() {
  pinMode(LH_INA, OUTPUT);
  pinMode(LH_INB, OUTPUT);
  pinMode(RH_INA, OUTPUT);
  pinMode(RH_INB, OUTPUT);
  
  pinMode(LH_ENCA, INPUT);
  pinMode(LH_ENCB, INPUT);
  pinMode(RH_ENCA, INPUT);
  pinMode(RH_ENCB, INPUT);
  
  attachInterrupt(digitalPinToInterrupt(LH_ENCA), lhInterrupt, RISING);
  attachInterrupt(digitalPinToInterrupt(RH_ENCA), rhInterrupt, RISING);
  
  Serial.begin(115200);
}


void loop() {

  time_curr = millis();

  // Control
  if (time_curr - time_prev >= INTERVAL_CONTROL) {
    time_prev = time_curr;
    
    // Motor 
    analogWrite(LH_INA, lh_pwm);
    analogWrite(RH_INA, rh_pwm);
    digitalWrite(LH_INB, lh_dir);
    digitalWrite(RH_INB, rh_dir);

    // Speed Calculating (Left-side)
    rev_motor = lh_encA_cnt / ENC_CHANGE_PER_REV;
    rev_wheel = rev_motor / GEAR_RATIO;
    dist_meas_curr = rev_wheel * 2 * PI * RADIUS;
    speed_meas = (dist_meas_curr - dist_meas_prev) / (INTERVAL_CONTROL / 1000); // Unit: cm/s

    dist_meas_prev = dist_meas_curr;
  }

  // Serial i/o
  if(time_curr - print_time_prev >= INTERVAL_PRINT) {
    print_time_prev = time_curr;

    Serial.print("Speed(cm/s): ");
    Serial.println(speed_meas);

    if (Serial.available()) {
      String input = Serial.readString();
      int input_int = input.toInt();
      Serial.print("PWM value changed to ");
      Serial.println(input_int);
      lh_pwm = input_int;
      rh_pwm = input_int;
    }
  }
}

void lhInterrupt()
{
  if (digitalRead(ENCA) == HIGH) {
    if (digitalRead(ENCB) == LOW) {
      lh_encA_cnt++;
    }
    else {
      lh_encA_cnt--;
    }
  }
  else {
    if (digitalRead(ENCB) == LOW) {
      lh_encA_cnt--;
    }
    else {
      lh_encA_cnt++;
    }
  }
}

void rhInterrupt()
{
  if (digitalRead(ENCA) == HIGH) {
    if (digitalRead(ENCB) == LOW) {
      rh_encA_cnt++;
    }
    else {
      rh_encA_cnt--;
    }
  }
  else {
    if (digitalRead(ENCB) == LOW) {
      rh_encA_cnt--;
    }
    else {
      rh_encA_cnt++;
    }
  }
}
