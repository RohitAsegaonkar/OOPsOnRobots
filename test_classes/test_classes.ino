  #include "Encoder.h"
#include "Mpu.h"
#include "Motor.h"
#include "Manual.h"
#include "Piston.h"

Manual A;
Encoder X(21, 50);
Encoder Y(2, 52);
Mpu V;
Motor M1(34, A.dirW1, 9);
Motor M2(28, A.dirW2, 6);
Motor M3(30, A.dirW3, 7);
Piston Gripper(41, 39), Throwing(37, 35);

int Piston_Press_Event = 0;

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
  A.Yaw = V.readMpu(3);
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
  pinMode(42,OUTPUT);
  pinMode(44,OUTPUT);
  pinMode(46,OUTPUT);
  pinMode(48,OUTPUT);
  
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
      A.forwardManY(A.Kp_strm2_forward, A.Kp_strm3_forward, A.Kp_encoder_forward);
      Serial.println("In F");
      break;

    case 'B':
      A.backwardManY(A.Kp_strm2_back, A.Kp_strm3_back, A.Kp_encoder_back);
      Serial.println("In B");
      break;   

    case 'L':
      A.leftManX(A.Kp_strm1_left,A.Kp_strm2_left, A.Kp_strm3_left, A.Kp_encoder_left);
      Serial.println("In L");
      break;

    case 'R':
      A.rightManX(A.Kp_strm1_left,A.Kp_strm2_left, A.Kp_strm3_left, A.Kp_encoder_left);
      Serial.println("In R");
      break;

    case 'S':
      M1.SetSpeed(0);
      M2.SetSpeed(0);
      M3.SetSpeed(0);
      digitalWrite(37,0);
      digitalWrite(39,0);
      digitalWrite(35,0);
      digitalWrite(41,0);
      Serial.println("In s");
      break;

    case 's':
      M1.SetSpeed(0);
      M2.SetSpeed(0);
      M3.SetSpeed(0);
      digitalWrite(37,0);
      digitalWrite(39,0);
      digitalWrite(35,0);
      digitalWrite(41,0);
      Serial.println("In s");
      break;

      case 'g':
      A.TurnMan(A.kp_ori, A.ki_ang, 15.00, 0);
      reset();
      break;

    case 'p':
      A.TurnMan(A.kp_ori, A.ki_ang, 15.00, 1);
      reset();
      break;

    case 'c':
      A.TTP_Man(0);
      break;

    case 'd':
      A.TTP_Man(1);
      break;

    case 'y':
    if (prevCommand =='S') 
    {
      Piston_Press_Event++;
    }
      Piston_Press_Event %= 3;
      if(Piston_Press_Event == 0)
      {
        Gripper.Retract(); 
        Serial.print("Piston_Press_Event:");
        Serial.println(Piston_Press_Event);
      }
      if(Piston_Press_Event == 1)
      {
        Throwing.Extend();
        Serial.print("Piston_Press_Event:");
        Serial.println(Piston_Press_Event);
      }
      if(Piston_Press_Event == 2)
      {
        Throwing.Retract();
        Gripper.Extend();
        Serial.print("Piston_Press_Event:");
        Serial.println(Piston_Press_Event);
      }
    break;
    
    case 'a':
    if (prevCommand =='S') 
    {
      Piston_Press_Event--;
    } 
      Piston_Press_Event %= 3;
      if(Piston_Press_Event == 0)
      {
        Gripper.Extend();
        Serial.print("Piston_Press_Event:");
        Serial.println(Piston_Press_Event);
      }
      if(Piston_Press_Event == 1)
      {
        Throwing.Extend();
        Serial.print("Piston_Press_Event:");
        Serial.println(Piston_Press_Event);
      }
      if(Piston_Press_Event == 2)
      {
        Throwing.Retract();
        Gripper.Retract(); 
        Serial.print("Piston_Press_Event:");
        Serial.println(Piston_Press_Event);  
      }
    
      break;

    case 'b':
      //Gripper.Retract();
      break;

    case 'x':
      //Gripper.Extend();
      break;

    default:
      M1.SetSpeed(0);
      M2.SetSpeed(0);
      M3.SetSpeed(0);

      

      Serial.println("In Default");
  }
  prevCommand = command;
}
