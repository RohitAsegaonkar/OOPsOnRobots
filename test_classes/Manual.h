#ifndef _Manual_H_
#define _Manual_H_

#include "Encoder.h"
#include "Mpu.h"
#include "Motor.h"

class Manual
{
  private:

    /******************************************* Creation of Objects ******************************************************/
    Encoder _X;
    Encoder _Y;
    Mpu _V;
    Motor _M1, _M2, _M3;
    /**********************************************************************************************************************/

    /********************************************** MPU Variables *********************************************************/
    float Yaw = 0;                                                                          //Variable to store the resolved value of yaw from -180 to 180
    float Shifted_Yaw = 0;                                                                  //Variable to store the changed/shifted value of yaw as set point for next function
    /**********************************************************************************************************************/

    /********************************************* PWM Values/Variables ***************************************************/
    #define Maxpwm 200.00
    #define basePwm 150
    int pwmm1, pwmm2, pwmm3;
    /**********************************************************************************************************************/

    /************************************* forwardManY() Function Variables ***********************************************/
    float error_encoder_forward;                                                            //Variable to store the value of the_X-Enc encoder as error.
    float error_forward;                                                                    //Variable to store error due to MPU
    float pwm_encoder_forward;                                                              //Variable to store PWM value due to error in encoder value   
    float error_sum_forward;                                                                //Variable to store sum of errors in encoder value 
    /**********************************************************************************************************************/

    /************************************** backwardManY() Function Variables *********************************************/
    float error_back;                                                                       //Variable to store error due to MPU
    float error_encoder_back;                                                               //Variable to store encoder value
    float pwm_encoder_back;                                                                 //Variable to store PWM value due to error in encoder value
    float error_sum_back;                                                                   //Variable to store sum of errors in encoder value 
    /**********************************************************************************************************************/

    /*************************************** leftManX() Function Variables ************************************************/
    float error_left;                                                                       //Variable to store error due to MPU                                   
    float error_encoder_left;                                                               //Variable to store encoder value
    float pwm_encoder_left;                                                                 //Variable to store PWM value due to error in encoder value
    float error_sum_left;                                                                   //Variable to store sum of errors in encoder value 
    /**********************************************************************************************************************/

    /****************************************** rightManX() Function Variables ********************************************/
    float error_right;                                                                      //Variable to store error due to MPU
    float error_encoder_right;                                                              //Variable to store encoder value
    float pwm_encoder_right;                                                                //Variable to store PWM value due to error in encoder value  
    float error_sum_right;                                                                  //Variable to store sum of errors in encoder value 
    /**********************************************************************************************************************/

    /******************************************** TurnMan() Function Variables ********************************************/
    float req_angle = 0.0;                                                                  //Variable to store the required angle for orientation control
    float error_ang = 3;                                                                    //Variable to store error in required angle and current angle
    float error_sum_ori;                                                                    //Variable to store sum of errors in required angle and current angle
    float prev_error = 0;                                                                   //Variable to store the error of previous itteration 
    /**********************************************************************************************************************/

  public:

    Manual()
    {
      Yaw = 0;
    }

    Manual(Motor m1, Motor m2, Motor m3, Mpu v, Encoder x, Encoder y)
    {
      /******************************************** Assigning the Motor Object ********************************************/
      _M1 = m1;
      _M2 = m2;
      _M3 = m3;
      /********************************************************************************************************************/

      /******************************************** Assigning the Encoder Object ******************************************/
      _X = x;
      _Y = y;
      /********************************************************************************************************************/

      //********************************************* Assigning the MPU object ********************************************/
      _V = v;
      /********************************************************************************************************************/
    }

    int final_ang = 0, correction_angle = 0;
    int count = 0;

    //PWM Given to motors for rotating
    float pwmm_ori;

    int pwmm_ori1, pwmm_ori2, pwmm_ori3;                                                            

/*
    Function Name       : UpdateShiftedYaw(float Yaw_ref)
    Input               : Yaw_ref
    Output              : Resets all the Variables
    Example Call        : UpdateShiftedYaw(Yaw_ref)
*/
void UpdateShiftedYaw(float Yaw_ref)
{
  Shifted_Yaw = Yaw_ref;
}

/*
    Function Name       : reset()
    Input               : None
    Output              : Resets all the Variables
    Example Call        : reset()
*/
void reset()
{
  _Y.encodervalue = 0;
  _X.encodervalue = 0;

  Serial.print("In Reset");
  Serial.print("\tA.Yaw = ");  
  Serial.println(_V.readMpu(2));

  UpdateShiftedYaw(_V.readMpu(2));

  pwmm1 = 0;
  pwmm2 = 0;
  pwmm3 = 0;
  prev_error = 0;
}

/*
    Function Name       : forwardManY()
    Input               : Proportionality constants of Each Motors(kp_strm2_forward,kp_strm3_forward),
                          Encoders(kp_encoder_forward)
    Output              : Motor 2 and 3 will move with same pwm and Motor 1 will have zero PWM. Bot will move in forward Direction(along Motor 1).
    Logic               : By giving same PWM to Motors 2 and 3 and Motor 1 will have zero PWM,the bot will move in forward direction. MPU and two XY Encoders are used for FEEDBACK of System.
                          PID is implemented for precise movement.
    Example Call        : forwardManY(float kp_strm2_forward,float kp_strm3_forward,float kp_encoder_forward);
*/
void forwardManY(float kp_strm2_forward, float kp_strm3_forward, float kp_encoder_forward)
{
  Serial.println("In F");
  Yaw = _V.readMpu(2);                                                                      // Reading Mpu Values
  error_forward = Yaw - Shifted_Yaw;                                                        // Calculating the angular shift of the bot.

  error_encoder_forward = _X.encodervalue;                                                  // Storing the encoder value of X - Encoder as bot's lateral shift
  pwm_encoder_forward = kp_encoder_forward * (error_encoder_forward);                       // Calculating the PWM according to the error in X - Encoder

  pwmm2 = basePwm + kp_strm2_forward * (error_forward) - pwm_encoder_forward ;              // Calculating the pwm for motor 2 according to the equations of velocities
  pwmm3 = basePwm - kp_strm3_forward * (error_forward) + pwm_encoder_forward + 6;           // Calculating the pwm for motor 3 according to the equations of velocities

  /*if (error_forward < 5)
    {
      pwmm3 = pwmm3 + 8;
    }*/

  if (pwmm2 > Maxpwm)
    pwmm2 = Maxpwm;

  if (pwmm3 > Maxpwm)
    pwmm3 = Maxpwm;

  _M1.SetDirection(0);
  _M2.SetDirection(1);
  _M3.SetDirection(1);

  _M1.SetSpeed(0);
  _M2.SetSpeed(abs(pwmm2));
  _M3.SetSpeed(abs(pwmm3));

  /********************************************* SERIAL PRINTING DATA ***************************************************/


  Serial.print("\tYaw: ");
  Serial.print(Yaw);
  Serial.print("\tShifted Yaw: ");
  Serial.print(Shifted_Yaw);
  Serial.print("\tError: ");
  Serial.print(error_forward);
  Serial.print("\tError encoder: ");
  Serial.print(error_encoder_forward);
  Serial.print("\tencodervalue1 :      ");
  Serial.print(_Y.encodervalue);
  Serial.print("\tkp_strm2_forward :      ");
  Serial.print(kp_strm2_forward);
  Serial.print("\tPWM:  ");
  Serial.print(pwmm1);
  Serial.print("   ");
  Serial.print(pwmm2);
  Serial.print("   ");
  Serial.println(pwmm3);

}

/*
    Function Name    :  backwardManY()
    Input            :  Proportionality constants of Each Motors(KP_M2_Backward,KP_M3_Backward),
                        Encoders(KP_Enc_Backward).
    Output           :  Motor 2 and 3 will move with same pwm and Motor 1 will have zero PWM. Bot will move in back Direction(along Motor 1 backward).
    Logic            :  By giving same PWM to Motors 2 and 3 and Motor 1 will have zero PWM,the bot will move in backward direction. MPU and two XY Encoders are used for FEEDBACK of System.
                        PID is implemented for precise movement.
    Example Call     :  backwardManY(float KP_M2_Backward, float KP_M3_Backward, float KP_Enc_Backward);
*/
void backwardManY(float KP_M2_Backward, float KP_M3_Backward, float KP_Enc_Backward)
{
  Yaw = _V.readMpu(2);                                                                     // Reading Mpu Values
  error_back = Yaw - Shifted_Yaw;                                                           
  error_encoder_back = _X.encodervalue;
  pwm_encoder_back = KP_Enc_Backward * (error_encoder_back);

  pwmm2 = basePwm - KP_M2_Backward * (error_back) - pwm_encoder_back;
  pwmm3 = basePwm + KP_M3_Backward * (error_back) + pwm_encoder_back;

  if (pwmm2 > Maxpwm)
    pwmm2 = Maxpwm;

  if (pwmm3 > Maxpwm)
    pwmm3 = Maxpwm;

  _M1.SetDirection(1);
  _M2.SetDirection(0);
  _M3.SetDirection(0);

  _M1.SetSpeed(0);
  _M2.SetSpeed(abs(pwmm2));
  _M3.SetSpeed(abs(pwmm3));

  /********************************************* SERIAL PRINTING DATA ***************************************************/

  Serial.print("\tYaw: ");
  Serial.print(Yaw);
  Serial.print("\tShifted Yaw: ");
  Serial.print(Shifted_Yaw);
  Serial.print("\tError: ");
  Serial.print(error_forward);
  Serial.print("\tError encoder: ");
  Serial.print(error_encoder_forward);
  Serial.print("\tencodervalue1 :      ");
  Serial.print(_Y.encodervalue);
  // Serial.print("\tkp_strm2_forward :      ");
  // Serial.print(kp_strm2_forward);
  Serial.print("\tPWM:  ");
  Serial.print(pwmm1);
  Serial.print("   ");
  Serial.print(pwmm2);
  Serial.print("   ");
  Serial.println(pwmm3);

}

/*
    Function Name         : leftManX()
    Input                 : Proportionality constants of Each Motors(KP_M1_Left,KP_M2_Left,KP_M3_Left),
                            Encoders(KP_Enc_Left)
    Output                : Motor 2 and 3 will move with same pwm and Motor 1 will move with double the pwm of rest two Motors. Bot will move in Left Direction.
    Logic                 : By giving same PWM to Motors 2 and 3 and giving double PWM to Motor 1,the bot will move in left direction. MPU and two XY Encoders are used for FEEDBACK of System.
                            PID is implemented for precise movement.
    Example Call          : leftManX(float KP_M1_Left, float KP_M2_Left, float KP_M3_Left, float KP_Enc_Left);
*/
void leftManX(float KP_M1_Left, float KP_M2_Left, float KP_M3_Left, float KP_Enc_Left)
{
  Yaw = _V.readMpu(2);
  error_left = Yaw - Shifted_Yaw;                                                 //Calculate the angular shift of the bot.                       
  error_encoder_left = _Y.encodervalue;                                           //Error for locomotion in X direction is given by the y encoder
  pwm_encoder_left = KP_Enc_Left * (error_encoder_left);                          //Calculating the pwm error

  pwmm1 = basePwm - KP_M1_Left * (error_left) - pwm_encoder_left;                 //Calculating the pwm for motor 1 according to the equations of velocities
  pwmm2 = (basePwm + KP_M2_Left * (error_left) + pwm_encoder_left) / 2 + 10;      //Calculating the pwm for motor 2 according to the equations of velocities
  pwmm3 = (basePwm + KP_M3_Left * (error_left) + pwm_encoder_left) / 2 + 10;      //Calculating the pwm for motor 3 according to the equations of velocities

  if (pwmm1 > Maxpwm)                                                             //The pwm should not exceed the desired maximum pwm
    pwmm1 = Maxpwm;

  if (pwmm2 > Maxpwm)
    pwmm2 = Maxpwm / 2;

  if (pwmm3 > Maxpwm)
    pwmm3 = Maxpwm / 2;

  _M1.SetDirection(1);
  _M2.SetDirection(1);
  _M3.SetDirection(0);

  _M1.SetSpeed(abs(pwmm1));
  _M2.SetSpeed(abs(pwmm2));
  _M3.SetSpeed(abs(pwmm3));

  /********************************************* SERIAL PRINTING DATA ***************************************************/

  Serial.print("\tYaw: ");
  Serial.print(Yaw);
  Serial.print("\tShifted Yaw: ");
  Serial.print(Shifted_Yaw);
  Serial.print("\tError: ");
  Serial.print(error_left);
  Serial.print("\tError encoder: ");
  Serial.print(error_encoder_left);
  Serial.print("\tencodervalue1 :      ");
  Serial.print(_X.encodervalue);
  // Serial.print("\tkp_strm2_forward :      ");
  // Serial.print(kp_strm2_forward);
  Serial.print("\tPWM:  ");
  Serial.print(pwmm1);
  Serial.print("   ");
  Serial.print(pwmm2);
  Serial.print("   ");
  Serial.println(pwmm3);


}
/*
    Function Name        :  rightManX()
    Input                :  Proportionality constants of Each Motors(KP_M1_Right,KP_M2_Right,KP_M3_Right),
                            Encoders(KP_Enc_Right)
    Output               :  Motor 2 and 3 will move with same pwm and Motor 1 will move with double the pwm of rest two Motors. Bot will move in Right Direction.
    Logic                :  By giving same PWM to Motors 2 and 3 and giving double PWM to Motor 1,the bot will move in right direction. MPU and two XY Encoders are used for FEEDBACK of System.
                            PID is implemented for precise movement.
    Example Call         :  rightManX(float KP_M1_Right, float KP_M2_Right, float KP_M3_Right, float KP_Enc_Right);
*/
void rightManX(float KP_M1_Right, float KP_M2_Right, float KP_M3_Right, float KP_Enc_Right)
{
  Yaw = _V.readMpu(2);
  error_right = Yaw - Shifted_Yaw;                                                 //Calculate the angular shift of the bot.
  error_encoder_right = _Y.encodervalue;                                           //Error for locomotion in X direction is given by the y encoder
  pwm_encoder_right = KP_Enc_Right * (error_encoder_right);                        //Calculating the pwm error

  pwmm1 = basePwm + KP_M1_Right * (error_right) - pwm_encoder_right;               //Calculating the pwm for motor 1 according to the equations of velocities 
  pwmm2 = (basePwm - KP_M2_Right * (error_right) + pwm_encoder_right) / 2 + 12;    //Calculating the pwm for motor 2 according to the equations of velocities
  pwmm3 = (basePwm - KP_M3_Right * (error_right) + pwm_encoder_right) / 2 + 12;    //Calculating the pwm for motor 3 according to the equations of velocities

  if (pwmm1 > Maxpwm)
    pwmm1 = Maxpwm;

  if (pwmm2 > Maxpwm)
    pwmm2 = Maxpwm / 2;

  if (pwmm3 > Maxpwm)
    pwmm3 = Maxpwm / 2;

  _M1.SetDirection(0);
  _M2.SetDirection(0);
  _M3.SetDirection(1);

  _M1.SetSpeed(abs(pwmm1));
  _M2.SetSpeed(abs(pwmm2));
  _M3.SetSpeed(abs(pwmm3));

  /********************************************* SERIAL PRINTING DATA ***************************************************/
  Serial.print("\tYaw: ");
  Serial.print(Yaw);
  Serial.print("\tShifted Yaw: ");
  Serial.print(Shifted_Yaw);
  Serial.print("\tError: ");
  Serial.print(error_right);
  Serial.print("\tError encoder: ");
  Serial.print(error_encoder_right);
  Serial.print("\tencodervalue1 :      ");
  Serial.print(_X.encodervalue);
  // Serial.print("\tkp_strm2_forward :      ");
  // Serial.print(kp_strm2_right);
  Serial.print("\tPWM:  ");
  Serial.print(pwmm1);
  Serial.print("   ");
  Serial.print(pwmm2);
  Serial.print("   ");
  Serial.println(pwmm3);

}

/*  Function Name       : TTP_Man()
    Input               : Required direction ( dir )
    Output              : Bot rotates in the respective direction untill the button is pressed.
    Logic               : Bot rotates either clockwise or anti - clockwise according to the button pressed, all motors are given equal pwm.
    Example Call        : TTP_Man(int dir)
*/
void TTP_Man(int dir)
{
  if (dir == 0)
  {
      _M1.SetDirection(1);
      _M2.SetDirection(0);
      _M3.SetDirection(1);
  }
  else if (dir == 1)
  {
      _M1.SetDirection(0);
      _M2.SetDirection(1);
      _M3.SetDirection(0);
  }

  Serial.print("\tShifted Yaw: ");
  Serial.println(Shifted_Yaw);

  pwmm_ori = 50;

  /********************************************* SERIAL PRINTING DATA ***************************************************/
  /*
    Serial.print("Yaw: ");
    Serial.print(Yaw);
    Serial.print("\tError: ");
    Serial.print(error_ang);
    Serial.print("\tKp:  ");
    Serial.print(kp_ori);
    Serial.print("\tPWM:  ");
    Serial.print(pwmm_ori);
    Serial.print("\trate:  ");
    Serial.println(rate_change);
    Serial.print("\tprevious:  ");
    Serial.println(prev_error);
  */

  pwmm_ori1 = pwmm_ori;
  pwmm_ori2 = pwmm_ori;
  pwmm_ori3 = pwmm_ori;

  _M1.SetDirection();
  _M2.SetDirection();
  _M3.SetDirection();

  _M1.SetSpeed(pwmm_ori1);
  _M2.SetSpeed(pwmm_ori2);
  _M3.SetSpeed(pwmm_ori3);
}

/*  Function Name       : M_Turn()
    Input               : Required angle(req_ang) for which function is to be executed and  Proportionality constants(KP_Orient, KI_Angle)
    Output              : Bot orients as per given required angle(req_angle).
    Logic               : Bot orients at desired or required angle from initial position, all motors are given equal pwm until it acquires desired angle.
    Example Call        : M_Turn(KP_Orient, KI_Angle, 338.00, 1);
*/
void TurnMan(float KP_Orient, float KI_Angle, float req_angle, int dir)
{
  error_ang = 5;
    if (!dir)
    {
      _M1.SetDirection(0);
      _M2.SetDirection(1);
      _M3.SetDirection(0);
    }
    else
    {
      _M1.SetDirection(1);
      _M2.SetDirection(0);
      _M3.SetDirection(1);
    }

  while ( abs(error_ang) > 2)
  {
    // read from port 1, send to port 0:
    Yaw = _V.readMpu(2);

    final_ang  = pow(-1, !dir) * req_angle + Shifted_Yaw ;
    final_ang += ((final_ang < - 180) - (final_ang > 180)) * 360 ;

    error_ang = final_ang - Yaw ;
    error_ang += ((error_ang < -180) - (error_ang > 180)) * 360 ;

    error_sum_ori = error_sum_ori + error_ang;

    if(prev_error * error_ang >= 0){
      flag = 1;
    }
    

    if (prev_error * error_ang < 0 && flag == 1)
    {
        _M1.ToggleDirection();
        _M2.ToggleDirection();
        _M3.ToggleDirection();

        flag = 0;
        Serial.println(".........................................");
          Serial.print("\tYaw: ");
        Serial.print(Yaw);
          Serial.print("\tShifted Yaw: ");
          Serial.print(Shifted_Yaw);
        Serial.print("\tError: ");
        Serial.print(error_ang);
        Serial.print("\tFinal: ");
        Serial.print(final_ang);
        Serial.print("\tM1_dir:  ");  
        Serial.print(_M1.GetDirection());
        Serial.print("\tM2_dir:  ");  
        Serial.print(_M2.GetDirection());
        Serial.print("\tM3_dir:  ");  
        Serial.print(_M3.GetDirection());        
        Serial.print("\tprevious:  ");  
        Serial.print(prev_error);
        Serial.print("\tdir:  ");
        Serial.println(dir);
        Serial.println(".........................................");

    }


    pwmm_ori = abs(error_ang) * KP_Orient;

    /*         KI 
    if (abs(error_ang) < 20)
    {
      pwmm_ori += (KI_Angle * error_sum_ori);
    }
    */

    if (pwmm_ori > (Maxpwm / 3))
      pwmm_ori = (Maxpwm / 3);

    

    pwmm_ori1 = pwmm_ori;
    pwmm_ori2 = pwmm_ori;
    pwmm_ori3 = pwmm_ori;

    _M1.SetSpeed(pwmm_ori1);
    _M2.SetSpeed(pwmm_ori2);
    _M3.SetSpeed(pwmm_ori3);

    prev_error = error_ang;

    /********************************************* SERIAL PRINTING DATA ***************************************************/

        Serial.print("\tYaw: ");
        Serial.print(Yaw);
          Serial.print("\tShifted Yaw: ");
          Serial.print(Shifted_Yaw);
        Serial.print("\tError: ");
        Serial.print(error_ang);
        Serial.print("\tFinal: ");
        Serial.print(final_ang);
//        Serial.print("\tKp:  ");
//        Serial.print(KP_Orient);
//        Serial.print("\tPWM:  ");
//        Serial.print(pwmm_ori);
//        Serial.print("\trate:  ");
//        Serial.print(rate_change);
        Serial.print("\tM1_dir:  ");  
        Serial.print(_M1.GetDirection());
        Serial.print("\tM2_dir:  ");  
        Serial.print(_M2.GetDirection());
        Serial.print("\tM3_dir:  ");  
        Serial.print(_M3.GetDirection());        
        Serial.print("\tprevious:  ");  
        Serial.print(prev_error);
        Serial.print("\tdir:  ");
        Serial.println(dir);

  }

  _M1.SetSpeed(0);
  _M2.SetSpeed(0);
  _M3.SetSpeed(0);
}

};

#endif
