//#include "Motor.h"
#include "Encoder.h"

//Motor M1(34,0,9);
Encoder X(2,50);
Encoder Y(21,52);
  
void UpdateXEncoder()
{
  X.updateEncoder();
}

void UpdateYEncoder()
{
  Y.updateEncoder();
}

void setup() 
{  
  Serial.begin(115200);
//Motor M1(34,0,9);


  attachInterrupt(2, UpdateXEncoder, RISING);                                          // interrupt of encoder1 will be triggered at the rising edge
  attachInterrupt(21, UpdateYEncoder, RISING);                                          // interrupt of encoder2 will be triggered at the rising edge
}

void loop()
{
  Serial.print("EncoderX:-");
  Serial.print(X.encodervalue);
  Serial.print("    EncoderY:-");
  Serial.print(Y.encodervalue);
}
