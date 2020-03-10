#include "Encoder.h"
#include "Mpu.h"
#include "Motor.h"
#include "Manual.h"
#include "Piston.h"
#include "Autonomous.h"

#define manual 1
#define autonomous 0

Encoder X(2, 29);
Encoder Y(3, 31);

Mpu mpu;

Motor M1(26, 0, 6);
Motor M2(38, 1, 12);
Motor M3(36, 1, 11);

Manual A(M1, M2, M3, X, Y);
Autonomous Auto(M1, M2, M3, mpu, X, Y);

Piston Gripper(41, 39), Throwing(37, 35);

/****************************************** PID Constants for forward function ******************************************/
#define Kp_encoder_forward 0.0                    //Proportionality constant for the lateral error
#define Kp_strm2_forward 0.6                      //Proportionality constant for the angular error for motor 2
#define Kp_strm3_forward 0.35                     //Proportionality constant for the angular error for motor 3
/************************************************************************************************************************/

/****************************************** PID Constants for backward function *****************************************/
#define Kp_encoder_back 0.068
#define Kp_strm2_back 0.25
#define Kp_strm3_back 0.25
/************************************************************************************************************************/

/********************************************* PID Constants for left function ******************************************/
#define Kp_encoder_left 0.02
#define Kp_strm1_left 0.4
#define Kp_strm2_left 0.6
#define Kp_strm3_left 0.6
/************************************************************************************************************************/

/****************************************** PID Constants for right function *****************************************/
#define Kp_encoder_right 0.03
#define Kp_strm1_right 0.6
#define Kp_strm2_right 0.4
#define Kp_strm3_right 0.45
/*********************************************************************************************************************/

/********************************* PID Constants for orientation control functions ***********************************/
#define kp_ori 2.75
#define ki_ang 0.001

bool ThrowFlag = 0;
bool GripFlag = 0;
char Non_S_Char;
float Last_Yaw;

void Debug()
{
  mpu.DebugMpu(4, 5);       //pass 2 PWM Pins
  X.DebugEncoderX(7, 8);    //pass 2 PWM Pins
  Y.DebugEncoderY(9, 10);   //pass 2 PWM Pins

}

void UpdateXEncoder()
{
  X.updateEncoderX();
}

void UpdateYEncoder()
{
  Y.updateEncoderY();
}

//Variables to accept charater from Controller
char command;
char prevCommand = 'a';

void setup()
{
  Serial.begin(115200);
//  Serial1.begin(115200);
  Serial2.begin(115200);
  Serial3.begin(115200);

  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);

  /*
    pinMode(42,OUTPUT);
    pinMode(44,OUTPUT);
    pinMode(46,OUTPUT);
    pinMode(48,OUTPUT);
  */
  //attachInterrupt(0, UpdateXEncoder, RISING);
  //attachInterrupt(1, UpdateYEncoder, RISING);
  //void Debug();

}

void loop()
{
  Debug();
#if autonomous
  //Serial.println(X.encodervalue);
  //Auto.forwardAutoY(200, 0.48, 0.48, ((Maxpwm/200)), 0.33, 0.33,0.00, 50, 40);
  //Auto.backwardAutoY(200, 0.25, 0.25, (Maxpwm / 200), 0.0625, 0.0000);
  //Auto.rightAutoX(200,0.45,0.45,0.45,(Maxpwm / 200), 0.05, 0.003,0,10,10);
  Auto.leftAutoX(200,0.2,1,0.8,(Maxpwm / 200)-0.005, 0.00, 0.1,0.1,0,5,20,20);
  delay(2000);
#endif

#if manual
  if (Serial3.available())
  {
    command = Serial3.read();
  }
  //Serial.print(command);

  if (command != 'S' && command != 's')
  {
    //Serial.println("In Not S condition");
    Non_S_Char = command;
  }

  if ((prevCommand != command) && (command != 'p' || command != 'g'))
  {
    //Serial.println("In Reset condition");
    A.reset(mpu.readMpu(2));
  }

  switch (command)
  {
    case 'F':
      A.forwardManY(Kp_strm2_forward, Kp_strm3_forward, Kp_encoder_forward, mpu.readMpu(2));
      //Serial.println("In F");
      break;

    case 'B':
      A.backwardManY(Kp_strm2_back, Kp_strm3_back, Kp_encoder_back, mpu.readMpu(2));
      //Serial.println("In B");
      break;

    case 'L':
      A.leftManX(Kp_strm1_left, Kp_strm2_left, Kp_strm3_left, Kp_encoder_left,  mpu.readMpu(2));
      //Serial.println("In L");
      break;

    case 'R':
      A.rightManX(Kp_strm1_left, Kp_strm2_left, Kp_strm3_left, Kp_encoder_left,  mpu.readMpu(2));
      //Serial.println("In R");
      break;

    case 'S':
      M1.SetSpeed(0);
      M2.SetSpeed(0);
      M3.SetSpeed(0);
      digitalWrite(37, 0);
      digitalWrite(39, 0);
      digitalWrite(35, 0);
      digitalWrite(41, 0);
      //Serial.println("In s");
      break;

    case 's':
      M1.SetSpeed(0);
      M2.SetSpeed(0);
      M3.SetSpeed(0);
      digitalWrite(37, 0);
      digitalWrite(39, 0);
      digitalWrite(35, 0);
      digitalWrite(41, 0);
//      Serial.println("In s");
      break;

    case 'g':
      A.TurnMan(kp_ori, ki_ang, 15.00, 0,  mpu.readMpu(2));
      A.reset(mpu.readMpu(2));
      break;

    case 'p':
      A.TurnMan(kp_ori, ki_ang, 15.00, 1,  mpu.readMpu(2));
      A.reset(mpu.readMpu(2));
      break;

    case 'd':
      A.TTP_Man(0);
      break;

    case 'c':
      A.TTP_Man(1);
      break;

    case 'y':
    if(prevCommand == 's' || prevCommand == 'S')
      ThrowFlag = !(ThrowFlag);
      if(ThrowFlag)
        Throwing.Extend();
      else if(!ThrowFlag)
        Throwing.Retract();
//      Serial.print("ThrowFlag\t");
//      Serial.println(ThrowFlag);  
      break;

    case 'a':
    if(prevCommand == 's' || prevCommand == 'S')
      GripFlag = !(GripFlag);
      if(GripFlag)
        Gripper.Extend();
      else if(!GripFlag)
         Gripper.Retract();
//      Serial.print("GripFlag\t");
//      Serial.println(GripFlag);  
      break;

    case 'b':
      break;

    case 'x':
      break;

    default:
      M1.SetSpeed(0);
      M2.SetSpeed(0);
      M3.SetSpeed(0);

//      Serial.println("In Default");

  }

  prevCommand = command;

#endif
}
