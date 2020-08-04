/* 20-02-19 John Lee
 * Arduino Project: Inverted Pendulum (One-Wheeled Balancing Cart)
 * Arduino Mega 2560
 */

#include <Arduino.h>
#include <Esplora.h>

void rhInterrupt();
void lhInterrupt();

// IMU Sensor Pins, Constants, and Variables
HardwareSerial &ImuSerial = Serial3;  // RX3, TX3 for seral communication

const int SBUF_SIZE = 64;

char sbuf[SBUF_SIZE];
signed int sbuf_cnt = 0;

float euler[3] = {0,};
float pitch_curr = 0;
float pitch_prev = 0;
float angular_speed = 0;

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
const float INTERVAL_CONTROL = 10;
const float INTERVAL_PRINT = 1000;

// Variables
int rh_dir = 0;  // forward: 0, backward: 1
int rh_pwm = 0;  // 0~255
int lh_dir = 1 - rh_dir;
int lh_pwm = 0;

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

float voltage = 0;
float u1 = 0;
float u2 = 0;
float u3 = 0;
float u4 = 0;

void setup() {
  pinMode(RH_INA, OUTPUT);
  pinMode(RH_INB, OUTPUT);
  pinMode(LH_INA, OUTPUT);
  pinMode(LH_INB, OUTPUT);

  pinMode(RH_ENCA, INPUT);
  pinMode(RH_ENCB, INPUT);
  pinMode(LH_ENCA, INPUT);
  pinMode(LH_ENCB, INPUT);

  attachInterrupt(digitalPinToInterrupt(RH_ENCA), rhInterrupt, CHANGE); 
  attachInterrupt(digitalPinToInterrupt(LH_ENCA), lhInterrupt, CHANGE);
  
  Serial.begin(115200);
  ImuSerial.begin(115200);

  // IMU Sensor Calibaration
  ImuSerial.print("cmf");
  ImuSerial.print("caf");
  delay(1000);
}


void loop() {

  time_curr = millis();

  // Sensor Reading
  if (time_curr - sensor_time_prev >= INTERVAL_SENSOR) {
    sensor_time_prev = time_curr;
    
    // IMU Sensor reading
    if (EBimuAsciiParser(euler, 3)) {
      pitch_curr = euler[1];
    }
    angular_speed = (pitch_curr - pitch_prev) / (INTERVAL_SENSOR / 1000);  // Unit: rad/s

    pitch_prev = pitch_curr;

    // Speed Calculation (Right-side)
    rh_rev_motor = rh_encA_cnt / ENC_CHANGE_PER_REV;
    rh_rev_wheel = rh_rev_motor / GEAR_RATIO;
    rh_dist_meas_curr = rh_rev_wheel * 2 * PI * RADIUS;
    rh_speed_meas = (rh_dist_meas_curr - rh_dist_meas_prev) / (INTERVAL_SENSOR / 1000); // Unit: cm/s

    rh_dist_meas_prev = rh_dist_meas_curr;

    // Speed Calculation (Left-side)
    lh_rev_motor = -lh_encA_cnt / ENC_CHANGE_PER_REV;
    lh_rev_wheel = lh_rev_motor / GEAR_RATIO;
    lh_dist_meas_curr = lh_rev_wheel * 2 * PI * RADIUS;
    lh_speed_meas = (lh_dist_meas_curr - lh_dist_meas_prev) / (INTERVAL_SENSOR / 1000); // Unit: cm/s

    lh_dist_meas_prev = lh_dist_meas_curr;

  }

  // Control
  // *** Need to tune more ***
  if (time_curr - control_time_prev >= INTERVAL_CONTROL) {
    control_time_prev = time_curr;
    
    u1 = (rh_dist_meas_curr + lh_dist_meas_curr) / 2;
    u2 = (rh_speed_meas + lh_speed_meas) / 2;
    u3 = pitch_curr;
    u4 = angular_speed;

    const float k1 = -0.7071;
    const float k2 = -3.9614;
    const float k3 = 72232;
    const float k4 = 5321.3;

    const float MAX_VOLTAGE = 12;

    // Change pwm and dir using sensor value
    voltage = -(k1*u1 + k2*u2 + k3*u3 + k4*u4) / 100000 * 1.75;  // Something wrong
    lh_pwm = rh_pwm = voltage / MAX_VOLTAGE * 255;
    if (rh_pwm < 0) {
      lh_pwm = rh_pwm = -rh_pwm;
      rh_dir = 1;
      lh_dir = 1 - rh_dir;
    }
    else {
      rh_dir = 0;
      lh_dir = 1 - rh_dir;
    }

      // saturation
    if (rh_pwm > 255) {
      lh_pwm = rh_pwm = 255;
    }

    // PWM and DIR
    analogWrite(RH_INA, rh_pwm);
    analogWrite(LH_INA, lh_pwm);
    digitalWrite(RH_INB, rh_dir);
    digitalWrite(LH_INB, lh_dir);
  }

  // Serial I/O
  if(time_curr - print_time_prev >= INTERVAL_PRINT) {
    print_time_prev = time_curr;
    
    Serial.print("Speed(cm/s): ");
    Serial.print(rh_speed_meas);
    Serial.print(' ');
    Serial.println(lh_speed_meas);

    Serial.print("\r");
    Serial.print("Euler angle: ");
    Serial.print(euler[0]);   Serial.print(" ");
    Serial.print(euler[1]);   Serial.print(" ");
    Serial.print(euler[2]);   Serial.print("\n\r");

    Serial.print("Voltage: ");
    Serial.print(voltage);
    Serial.print("\n\r");

    Serial.print("u1, u2, u3, u4: ");
    Serial.print(u1);   Serial.print(" ");
    Serial.print(u2);   Serial.print(" ");
    Serial.print(u3);   Serial.print(" ");
    Serial.print(u4);   Serial.print(" ");
    Serial.print("\n\r");

    Serial.print("rh_pwm, lh_pwm: ");
    Serial.print(rh_pwm);   Serial.print(" ");
    Serial.print(lh_pwm);   Serial.print(" ");
    Serial.println("\n\r");
    
    if (Serial.available()) {
      String input = Serial.readString();
      int input_int = input.toInt();
      Serial.print("PWM value changed to ");
      Serial.println(input_int);
      Serial.print("\n\r");
      rh_pwm = input_int;
      lh_pwm = input_int;
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


int EBimuAsciiParser(float *item, int number_of_item)
{
  int n,i;
  int rbytes;
  char *addr; 
  int result = 0;
  
  rbytes = ImuSerial.available();
  for(n=0;n<rbytes;n++)
  {
    sbuf[sbuf_cnt] = ImuSerial.read();
    if(sbuf[sbuf_cnt]==0x0a)
       {
           addr = strtok(sbuf,",");
           for(i=0;i<number_of_item;i++)
           {
              item[i] = atof(addr);
              addr = strtok(NULL,",");
           }

           result = 1;
           /*
           Serial.print("\n\r");
           for(i=0;i<number_of_item;i++) {  
             Serial.print(item[i]);  
             Serial.print(" "); 
           } */
       }
     else if(sbuf[sbuf_cnt]=='*')
       {   sbuf_cnt=-1;
       }

     sbuf_cnt++;
     if(sbuf_cnt>=SBUF_SIZE) sbuf_cnt=0;
  }
  
  return result;
}
