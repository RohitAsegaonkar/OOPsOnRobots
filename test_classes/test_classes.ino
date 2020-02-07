#include "Encoder.h"
#include "Mpu.h"
#include "Motor.h"
#include "Manual.h"

  Manual A;
  Encoder X(21,50);
  Encoder Y(2,52);
  Mpu V;
  Motor M1(34,A.dirW1,9);
  Motor M2(28,A.dirW2,6);
  Motor M3(30,A.dirW3,7);

void UpdateXEncoder()
{
  X.updateEncoder();
}

void UpdateYEncoder()
{
  Y.updateEncoder();
}

//Variables to accept charater from Controller
char command;
char prevCommand = 'a';

void reset()
{
  Y.encodervalue = 0;
  X.encodervalue = 0;
  A.Yaw = V.readMpu();
  A.Shifted_Yaw = A.Yaw;
  Serial.println("In Reset");
  Serial.println(A.Shifted_Yaw);
  A.pwmm1 = 0;
  A.pwmm2 = 0;
  A.pwmm3 = 0;
}

void setup() 
{  
  Serial.begin(115200);
  Serial1.begin(115200);
  Serial3.begin(115200);
}

void loop()
{
if (Serial1.available())
  {
    command = Serial1.read();
  }
  Serial.print(command);

  if (prevCommand != command && (command != 'c' || command != 'd'))
  {
    reset();
  }

  switch (command)
  {
    case 'F':
      void forwardManY(float kp_strm2_forward, float kp_strm3_forward, float kp_encoder_forward);
      Serial.println("In F");
      break;

    case 'B':
      void backwardManY(float KP_M2_Backward, float KP_M3_Backward, float KP_Enc_Backward);
      Serial.println("In B");
      break;

    case 'L':
      void leftManX(float KP_M1_Left, float KP_M2_Left, float KP_M3_Left, float KP_Enc_Left);
      Serial.println("In L");
      break;

    case 'R':
      void rightManX(float KP_M1_Right, float KP_M2_Right, float KP_M3_Right, float KP_Enc_Right);
      Serial.println("In R");
      break;

    case 'S':
      analogWrite(A.pwmm1, 0);
      analogWrite(A.pwmm2, 0);
      analogWrite(A.pwmm3, 0);
      Serial.println("In s");
      break;

    case 's':
      analogWrite(A.pwmm1, 0);
      analogWrite(A.pwmm2, 0);
      analogWrite(A.pwmm3, 0);

      Serial.println("In s");
      break;

    default:
      analogWrite(A.pwmm1, 0);
      analogWrite(A.pwmm2, 0);
      analogWrite(A.pwmm3, 0);

      Serial.println("In Default");
  }
  prevCommand = command;
}
