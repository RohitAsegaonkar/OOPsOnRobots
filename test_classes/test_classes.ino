#include "Encoder.h"
#include "Mpu.h"
#include "Motor.h"
#include "Manual.h"
#include "Piston.h"
#include "Autonomous.h"

#define manual 0
#define autonomous 1

Encoder X(2, 29);
Encoder Y(3, 31);

Mpu mpu;

Motor M1(26, 0, 6);
Motor M2(38, 1, 12);
Motor M3(36, 1, 11);

Manual A(M1, M2, M3, mpu, X, Y);
Autonomous Auto(M1, M2, M3, mpu, &X, &Y);

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

int Piston_Press_Event = 0;
char Non_S_Char;
float Last_Yaw;

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
  Serial1.begin(115200);
  Serial2.begin(115200);
  Serial3.begin(115200);



  /*
    pinMode(42,OUTPUT);
    pinMode(44,OUTPUT);
    pinMode(46,OUTPUT);
    pinMode(48,OUTPUT);
  */
  attachInterrupt(0, UpdateXEncoder, RISING);
  attachInterrupt(1, UpdateYEncoder, RISING);
}

void loop()
{
#if autonomous
  //Serial.println(X.encodervalue);
  Auto.forwardAutoY(200, 0.48, 0.48, ((Maxpwm/200)), 0.33, 0.33,0.00, 50, 40);
  //Auto.backwardAutoY(200, 0.25, 0.25, (Maxpwm / 200), 0.0625, 0.0000);
  // Auto.rightAutoX(200,0.45,0.45,0.45,(Maxpwm / 200), 0.05, 0.003,0,10,10);
  delay(20000);
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
    A.reset();
  }

  switch (command)
  {
    case 'F':
      A.forwardManY(Kp_strm2_forward, Kp_strm3_forward, Kp_encoder_forward);
      //Serial.println("In F");
      break;

    case 'B':
      A.backwardManY(Kp_strm2_back, Kp_strm3_back, Kp_encoder_back);
      //Serial.println("In B");
      break;

    case 'L':
      A.leftManX(Kp_strm1_left, Kp_strm2_left, Kp_strm3_left, Kp_encoder_left);
      //Serial.println("In L");
      break;

    case 'R':
      A.rightManX(Kp_strm1_left, Kp_strm2_left, Kp_strm3_left, Kp_encoder_left);
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
      Serial.println("In s");
      break;

    case 'g':
      A.TurnMan(kp_ori, ki_ang, 15.00, 0);
      A.reset();
      break;

    case 'p':
      A.TurnMan(kp_ori, ki_ang, 15.00, 1);
      A.reset();
      break;

    case 'c':
      A.TTP_Man(0);
      break;

    case 'd':
      A.TTP_Man(1);
      break;

    case 'y':
      if (prevCommand == 'S')
      {
        Piston_Press_Event++;
      }
      Piston_Press_Event %= 3;
      if (Piston_Press_Event == 0)
      {
        Gripper.Retract();
        Serial.print("Piston_Press_Event:");
        Serial.println(Piston_Press_Event);
      }
      if (Piston_Press_Event == 1)
      {
        Throwing.Extend();
        Serial.print("Piston_Press_Event:");
        Serial.println(Piston_Press_Event);
      }
      if (Piston_Press_Event == 2)
      {
        Throwing.Retract();
        Gripper.Extend();
        Serial.print("Piston_Press_Event:");
        Serial.println(Piston_Press_Event);
      }
      break;

    case 'a':
      if (prevCommand == 'S')
      {
        Piston_Press_Event--;
      }
      Piston_Press_Event %= 3;
      if (Piston_Press_Event == 0)
      {
        Gripper.Extend();
        Serial.print("Piston_Press_Event:");
        Serial.println(Piston_Press_Event);
      }
      if (Piston_Press_Event == 1)
      {
        Throwing.Retract();
        Serial.print("Piston_Press_Event:");
        Serial.println(Piston_Press_Event);
      }
      if (Piston_Press_Event == 2)
      {
        Throwing.Extend();
        Gripper.Retract();
        Serial.print("Piston_Press_Event:");
        Serial.println(Piston_Press_Event);
      }
      break;

    case 'b':
      //Gripper.Retract();u
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

#endif
}
