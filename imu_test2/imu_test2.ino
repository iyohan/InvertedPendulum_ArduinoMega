////////////////////////////////////////////////////////////////////
/////////////////////// EBIMU FUNCTION /////////////////////////////
#define SBUF_SIZE 64

char sbuf[SBUF_SIZE];
signed int sbuf_cnt=0;

int EBimuAsciiParser(float *item, int number_of_item)
{
  int n,i;
  int rbytes;
  char *addr; 
  int result = 0;
  
  rbytes = Serial.available();
  for(n=0;n<rbytes;n++)
  {
    sbuf[sbuf_cnt] = Serial.read();
    if(sbuf[sbuf_cnt]==0x0a)
       {
           addr = strtok(sbuf,",");
           for(i=0;i<number_of_item;i++)
           {
              item[i] = atof(addr);
              addr = strtok(NULL,",");
           }

           result = 1;

         // Serial.print("\n\r");
         // for(i=0;i<number_of_item;i++)  {  Serial.print(item[i]);  Serial.print(" "); }
       }
     else if(sbuf[sbuf_cnt]=='*')
       {   sbuf_cnt=-1;
       }

     sbuf_cnt++;
     if(sbuf_cnt>=SBUF_SIZE) sbuf_cnt=0;
  }
  
  return result;
}
/////////////////////// EBIMU FUNCTION /////////////////////////////
////////////////////////////////////////////////////////////////////


void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  float euler[3];
  
  EBimuAsciiParser(euler, 3);
  
  if((euler[0]<5)&&(euler[0]>-5)&&(euler[1]<5)&&(euler[1]>-5)) digitalWrite(LED_BUILTIN, HIGH);
  else digitalWrite(LED_BUILTIN, LOW);
   
}
