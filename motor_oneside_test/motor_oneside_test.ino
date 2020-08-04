// http://makeshare.org/bbs/board.php?bo_table=arduinomotor&wr_id=12
// 모터 스피드 테스트 (Encoder Test)

const int INA = 8;  // EN (speed)
const int INB = 7;  // DIR

const int ENCA = 2;  // Encoder output A 
const int ENCB = 3;  // Encoder output B

const int GEAR_RATIO = 131.25;  // Gearbox ratio
const float RADIUS = 5.75;  // radius of wheel (cm)

int dir = 0;  // 0 or 1
int pwm = 128;  //0~255

unsigned long time_curr = 0;
unsigned long time_prev = 0;
unsigned long print_time_prev = 0;

volatile long encA_cnt = 0;

// Speed 계산용.
float rev_motor = 0;
float rev_wheel = 0;
float dist_meas_curr = 0;
float dist_meas_prev = 0;
float speed_meas = 0;
int dir_meas = 0;

float time_interval = 100; //ms
float print_interval = 1000;

void setup() {
  pinMode(INA, OUTPUT);
  pinMode(INB, OUTPUT);
  pinMode(ENCA, INPUT);
  pinMode(ENCB, INPUT);
  
  attachInterrupt(digitalPinToInterrupt(ENCA), interrupt, CHANGE);
  
  Serial.begin(115200);
}

void loop() {

  time_curr = millis();

  if (time_curr - time_prev >= time_interval) {
    time_prev = time_curr;
    
    // 모터 구동
    /*
    if (speed_analog < 255) {
      speed_analog += 1;
    }
    else {
      speed_analog = 255;
    }
    */
    analogWrite(INA, pwm);
    digitalWrite(INB, dir);

    // Speed Measuring
    rev_motor = encA_cnt / 32;
    rev_wheel = rev_motor / GEAR_RATIO;
    dist_meas_curr = rev_wheel * 2 * PI * RADIUS;
    speed_meas = (dist_meas_curr - dist_meas_prev) / (time_interval / 1000); // Unit: cm/s

    dist_meas_prev = dist_meas_curr;
  }

  if(time_curr - print_time_prev >= print_interval) {
    print_time_prev = time_curr;

    Serial.print("Speed(cm/s): ");
    Serial.println(speed_meas);

    if (Serial.available()) {
      String input = Serial.readString();
      int input_int = input.toInt();
      Serial.print("PWM value changed to ");
      Serial.println(input_int);
      pwm = input_int;
    }
  }
}

void interrupt()
{
  if (digitalRead(ENCA) == HIGH) {
    if (digitalRead(ENCB) == LOW) {
      encA_cnt++;
    }
    else {
      encA_cnt--;
    }
  }
  else {
    if (digitalRead(ENCB) == LOW) {
      encA_cnt--;
    }
    else {
      encA_cnt++;
    }
  }
  
}
