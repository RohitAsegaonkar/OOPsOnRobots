//#include "Motor.h"
#include "Encoder.h"

//Motor M1(34,0,9);
Encoder X(21,50);
Encoder Y(2,52);
  
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


  attachInterrupt(0, UpdateXEncoder, RISING);                                          // interrupt of encoder1 will be triggered at the rising edge
  attachInterrupt(2, UpdateYEncoder, RISING);                                          // interrupt of encoder2 will be triggered at the rising edge
}

void loop()
{
  Serial.print("EncoderX: ");
  Serial.print(X.encodervalue);
  Serial.print("    EncoderY: ");
  Serial.println(Y.encodervalue);
}
