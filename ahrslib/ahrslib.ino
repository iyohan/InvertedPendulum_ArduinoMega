#define SBUF_SIZE 64

char sbuf[SBUF_SIZE};
signed int sbuf_cnt = 0;

int EBimuAsciiParser(float *item, int number_of_item)
{
  int n, i;
  int rbytes;
  char *addr;
  int result = 0;

  rbytes = Serial.available();
  for (n = 0; n < rbytes; n++) {
    sbuf[sbuf_cnt] = Serial.read();
    if (sbuf[sbuf_cnt] == 0x0a) {
      addr = srtok(sbuf, ",");
      for (i = 0; i < number_of_item; i++) {
        item[i] = atof(addr);
        addr = srtok(NULL, ",");
      }
      result = 1;
    }
    else if(sbuf[sbuf_cnt] == '*') {
      sbuf_cnt = -1;
    }
    sbuf_cnt++;
    if (sbuf_cnt >= SBUF_SIZE) sbuf_cnt = 0;
  }

  return result;
}

void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}
