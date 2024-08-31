#include <SoftwareSerial.h>

SoftwareSerial mySerial(2, 3); // RX, TX
unsigned char receive_buffer[20];

void setup()
{
  Serial.begin(9600);
  mySerial.begin(9600);

  union
  {
    float f_val;
    unsigned char f_val_buffer[4];
  }bmp_temp,bmp_press,dht_temp,dht_hum;

  unsigned int ldr_voltage = 0, raindrops_voltage = 0;

}

void loop() // run over and over
{
  if (mySerial.available())
  {
    unsigned char receive_buffer[20];
    for(int i=0;i<20;i++)
    {
      receive_buffer[i] = mySerial.read();
    }
    
  }

  if(Serial.availableForWrite())
  {
    for(int i=0;i<20;i++)
    {
      Serial.write((int) receive_buffer[i]);
    }
  }
  /*
  for(int i=0;i<4;i++)
  {
    bmp_temp.f_val_buffer[i] = recevie_buffer[i];
    bmp_press.f_val_buffer[i] = receive_buffer[i+4];
    dht_temp.f_val_buffer[i] = receive_buffer[i+8];
    dht_temp.f_val_buffer[i] = receive_buffer[i+12];
  }

  ldr_voltage = (int) ((receive_buffer[17]<<8) | (receive_buffer[16]));
  raindrops_voltage = (int) ((receive_buffer[19]<<8) | (receive_buffer[18]));  
  */
}

