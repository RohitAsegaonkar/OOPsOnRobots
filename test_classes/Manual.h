#ifndef _Manual_H_
#define _Manual_H_

#include "Encoder.h"
#include "Mpu.h"
#include "Motor.h"


class Manual
{
  private:

  public:

    Manual(/* args */);
    //Boolean Variables to set direction of Wheels
    bool dirW1 = 0 ;
    bool dirW2 ;
    bool dirW3 ;
    int Yaw;                                          //Variable to store the resolved value of yaw from -180 to 180
    float Shifted_Yaw;

    /************ PWM Values ************/
#define Maxpwm 150.00
#define basePwm 100
    int pwmm1, pwmm2, pwmm3;

    /************************ Objects Created *********************************/
    Encoder X;
    Encoder Y;
    Mpu V;

    //Motor M1;
    // Motor M1(34, dirW1, 9);
    //Motor M2(28, dirW2, 6);
    //Motor M3(30, dirW3, 7);
    Motor M1, M2, M3;

    void forwardManY(float kp_strm2_forward, float kp_strm3_forward, float kp_encoder_forward);
    void backwardManY(float KP_M2_Backward, float KP_M3_Backward, float KP_Enc_Backward);
    void leftManX(float KP_M1_Left, float KP_M2_Left, float KP_M3_Left, float KP_Enc_Left);
    void rightManX(float KP_M1_Right, float KP_M2_Right, float KP_M3_Right, float KP_Enc_Right);
    void TurnMan(float KP_Orient, float KI_Angle, float req_angle, int dir);
    void TTP_Man(int dir);


    float error_forward;                                 //Variable to store the value of the X encoder as error.
    const float Kp_encoder_forward = 0.0 ;                       //Proportionality constant for the lateral error
    const float Kp_strm2_forward = 0.48 ;                       //Proportionality constant for the angular error for motor 2
    const float  Kp_strm3_forward = 0.48 ;                        //Proportionality constant for the angular error for motor 3

    float error_encoder_forward;
    float pwm_encoder_forward;
    float error_sum_forward;

    /*** backwardManY() Function Variables ***/
    float error_back;
    const float Kp_encoder_back = 0.068;
    const float Kp_strm2_back = 0.25;
    const float Kp_strm3_back = 0.25;

    float error_encoder_back;
    float pwm_encoder_back;
    float error_sum_back;

    /***** leftManX() Function Variables *****/
    float error_left;                                     //Variable to store the value of the X encoder as error.
    const float Kp_encoder_left = 0.02;
    const float Kp_strm1_left = 0.4;
    const float Kp_strm2_left = 0.6;
    const float Kp_strm3_left = 0.6;

    float error_encoder_left;
    float pwm_encoder_left;
    float error_sum_left;

    /***** rightManX() Function Variables *****/
    float error_right;
    const float Kp_encoder_right = 0.03;
    const float Kp_strm1_right = 0.4;
    const float Kp_strm2_right = 0.4;
    const float Kp_strm3_right = 0.45;

    float error_encoder_right;
    float pwm_encoder_right;
    float error_sum_right;

    int final_ang = 0, correction_angle = 0;
    //PWM Given to motors for rotating
    float pwmm_ori;

    //PWM Values given to each wheel
    int pwmm_ori1, pwmm_ori2, pwmm_ori3;

    //Variables used for PID
    float req_angle = 0.0;
    float error_ang = 3;

    //Variables required to keep track of future and past
    float rate_change = 5;
    float error_sum_ori;
    float prev_error = 0;

    //PID Constants
    const float kp_ori = 2.75;
    const float ki_ang = 0.000;


};


Manual::Manual(/* args */)
{
  Yaw = 0;
  Shifted_Yaw = 0;
}

/*
    Function Name       : forwardManY()
    Input               : Required distance(requiredDistance_forward) for which function is to be executed,Proportionality constants of Each Motors(kp_strm1_forward,KP_M2_Forward,KP_M3_Forward),
                          Encoders(KP_Enc_Forward) and Distance constant(kp_dist_forward,ki_dist_forward)
    Output              : Motor 2 and 3 will move with same pwm and Motor 1 will have zero PWM. Bot will move in forward Direction(along Motor 1).
    Logic               : By giving same PWM to Motors 2 and 3 and Motor 1 will have zero PWM,the bot will move in forward direction. MPU and two XY Encoders are used for FEEDBACK of System.
                          PID is implemented for precise movement.
    Example Call        : forwardManY(float kp_strm2_forward,float kp_strm3_forward,float kp_encoder_forward);
*/
void Manual :: forwardManY(float kp_strm2_forward, float kp_strm3_forward, float kp_encoder_forward)
{
  Motor M1(34, 1, 9);
  Motor M2(28, 1, 6);
  Motor M3(30, 1, 7);

  Serial.println("In F");
  Yaw = V.readMpu();                          // Reading Mpu Values
  error_forward = Yaw - Shifted_Yaw;

  error_encoder_forward = X.encodervalue;
  pwm_encoder_forward = kp_encoder_forward * (error_encoder_forward);

  pwmm2 = basePwm + kp_strm2_forward * (error_forward) - pwm_encoder_forward ;
  pwmm3 = basePwm - kp_strm3_forward * (error_forward) + pwm_encoder_forward + 6;

  /*if (error_forward < 5)
    {
      pwmm3 = pwmm3 + 8;
    }*/

  if (pwmm2 > Maxpwm)
    pwmm2 = Maxpwm;

  if (pwmm3 > Maxpwm)
    pwmm3 = Maxpwm;

  M1.SetDirection();
  M2.SetDirection();
  M3.SetDirection();

  M1.SetSpeed(0);
  M2.SetSpeed(abs(pwmm2));
  M3.SetSpeed(abs(pwmm3));

  /********************************************* SERIAL PRINTING DATA ***************************************************/


  Serial.print("\tYaw: ");
  Serial.print(Yaw);
  Serial.print("\tError: ");
  Serial.print(error_forward);
  Serial.print("\tError encoder: ");
  Serial.print(error_encoder_forward);
  Serial.print("\tencodervalue1 :      ");
  Serial.print(Y.encodervalue);
  Serial.print("\tPWM:  ");
  Serial.print(pwmm1);
  Serial.print("   ");
  Serial.print(pwmm2);
  Serial.print("   ");
  Serial.println(pwmm3);

}


/*
    Function Name    :  backwardManY()
    Input            :  Required distance(requiredDistance_back) for which function is to be executed,Proportionality constants of Each Motors(kp_strm1_back,KP_M2_Backward,KP_M3_Backward),
                        Encoders(KP_Enc_Backward) and Distance constant(kp_dist_back,ki_dist_back)
    Output           :  Motor 2 and 3 will move with same pwm and Motor 1 will have zero PWM. Bot will move in back Direction(along Motor 1 backward).
    Logic            :  By giving same PWM to Motors 2 and 3 and Motor 1 will have zero PWM,the bot will move in backward direction. MPU and two XY Encoders are used for FEEDBACK of System.
                        PID is implemented for precise movement.
    Example Call     :  backwardManY(float KP_M2_Backward, float KP_M3_Backward, float KP_Enc_Backward);
*/
void Manual :: backwardManY(float KP_M2_Backward, float KP_M3_Backward, float KP_Enc_Backward)
{

  Motor M1(34, 1, 9);
  Motor M2(28, 0, 6);
  Motor M3(30, 0, 7);

  Yaw = V.readMpu();                         // Reading Mpu Values
  error_back = Yaw - Shifted_Yaw;

  error_encoder_back = X.encodervalue;
  pwm_encoder_back = KP_Enc_Backward * (error_encoder_back);

  pwmm2 = basePwm - KP_M2_Backward * (error_back) - pwm_encoder_back;
  pwmm3 = basePwm + KP_M3_Backward * (error_back) + pwm_encoder_back;

  if (pwmm2 > Maxpwm)
    pwmm2 = Maxpwm;

  if (pwmm3 > Maxpwm)
    pwmm3 = Maxpwm;

  M1.SetDirection();
  M2.SetDirection();
  M3.SetDirection();

  M1.SetSpeed(0);
  M2.SetSpeed(abs(pwmm2));
  M3.SetSpeed(abs(pwmm3));

  /********************************************* SERIAL PRINTING DATA ***************************************************/


  Serial.print("\tYaw: ");
  Serial.print(Yaw);
  Serial.print("\tError: ");
  Serial.print(error_back);
  Serial.print("\tError encoder: ");
  Serial.print(error_encoder_back);
  Serial.print("\tencodervalue2 :      ");
  Serial.print(Y.encodervalue);
  Serial.print("\tPWM:  ");
  Serial.print(pwmm1);
  Serial.print("   ");
  Serial.print(pwmm2);
  Serial.print("   ");
  Serial.println(pwmm3);
}

/*
    Function Name         : leftManX()
    Input                 : Required distance(requiredDistance_left) for which function is to be executed,Proportionality constants of Each Motors(KP_M1_Left,KP_M2_Left,KP_M3_Left),
                            Encoders(KP_Enc_Left) and Distance constant(kp_dist_left,ki_dist_left)
    Output                : Motor 2 and 3 will move with same pwm and Motor 1 will move with double the pwm of rest two Motors. Bot will move in Left Direction.
    Logic                 : By giving same PWM to Motors 2 and 3 and giving double PWM to Motor 1,the bot will move in left direction. MPU and two XY Encoders are used for FEEDBACK of System.
                            PID is implemented for precise movement.
    Example Call          : leftManX(float KP_M1_Left, float KP_M2_Left, float KP_M3_Left, float KP_Enc_Left);
*/
void Manual :: leftManX(float KP_M1_Left, float KP_M2_Left, float KP_M3_Left, float KP_Enc_Left)
{
  Motor M1(34, 1, 9);
  Motor M2(28, 1, 6);
  Motor M3(30, 0, 7);

  Yaw = V.readMpu();
  error_left = Yaw - Shifted_Yaw;                                                 //Calculate the angular shift of the bot. Yaw_ref is the reference yaw value from the previous function                       //Calculating the basepwm in proportion with the error

  error_encoder_left = Y.encodervalue;                                            //Error for locomotion in X direction is given by the y encoder
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

  M1.SetDirection();
  M2.SetDirection();
  M3.SetDirection();


  M1.SetSpeed(abs(pwmm1));
  M2.SetSpeed(abs(pwmm2));
  M3.SetSpeed(abs(pwmm3));

  /********************************************* SERIAL PRINTING DATA ***************************************************/

  Serial.print("\tYaw: ");
  Serial.print(Yaw);
  Serial.print("\tError: ");
  Serial.print(error_left);
  Serial.print("\tError encoder: ");
  Serial.print(error_encoder_left);
  Serial.print("\tencodervalue2 :      ");
  Serial.print(X.encodervalue);
  Serial.print("\tPWM:  ");
  Serial.print(pwmm1);
  Serial.print("   ");
  Serial.print(pwmm2);
  Serial.print("   ");
  Serial.println(pwmm3);
  Serial.print("\tBasepwm: ");
  Serial.print(basePwm);

}
/*
    Function Name        :  rightManX()
    Input                :  Required distance(requiredDistance_right) for which function is to be executed,Proportionality constants of Each Motors(KP_M1_Right,KP_M2_Right,KP_M3_Right),
                            Encoders(KP_Enc_Right) and Distance constant(kp_dist_right,ki_dist_right)
    Output               :  Motor 2 and 3 will move with same pwm and Motor 1 will move with double the pwm of rest two Motors. Bot will move in Right Direction.
    Logic                :  By giving same PWM to Motors 2 and 3 and giving double PWM to Motor 1,the bot will move in right direction. MPU and two XY Encoders are used for FEEDBACK of System.
                            PID is implemented for precise movement.
    Example Call         :  rightManX(float KP_M1_Right, float KP_M2_Right, float KP_M3_Right, float KP_Enc_Right);
*/
void Manual :: rightManX(float KP_M1_Right, float KP_M2_Right, float KP_M3_Right, float KP_Enc_Right)
{
  Motor M1(34, 0, 9);
  Motor M2(28, 0, 6);
  Motor M3(30, 1, 7);
  Yaw = V.readMpu();

  error_encoder_right = Y.encodervalue;
  pwm_encoder_right = KP_Enc_Right * (error_encoder_right);

  pwmm1 = basePwm + KP_M1_Right * (error_right) - pwm_encoder_right;
  pwmm2 = (basePwm - KP_M2_Right * (error_right) + pwm_encoder_right) / 2 + 12;
  pwmm3 = (basePwm - KP_M3_Right * (error_right) + pwm_encoder_right) / 2 + 12;

  if (pwmm1 > Maxpwm)
    pwmm1 = Maxpwm;

  if (pwmm2 > Maxpwm)
    pwmm2 = Maxpwm / 2;

  if (pwmm3 > Maxpwm)
    pwmm3 = Maxpwm / 2;

  M1.SetDirection();
  M2.SetDirection();
  M3.SetDirection();

  M1.SetSpeed(abs(pwmm1));
  M2.SetSpeed(abs(pwmm2));
  M3.SetSpeed(abs(pwmm3));

  /********************************************* SERIAL PRINTING DATA ***************************************************/

  Serial.print("\tYaw: ");
  Serial.print(Yaw);
  Serial.print("\tError: ");
  Serial.print(error_right);
  Serial.print("\tError encoder: ");
  Serial.print(error_encoder_right);
  Serial.print("\tencodervalue2 :      ");
  Serial.print(X.encodervalue);
  Serial.print("\tPWM:  ");
  Serial.print(pwmm1);
  Serial.print("   ");
  Serial.print(pwmm2);
  Serial.print("   ");
  Serial.println(pwmm3);
  Serial.print("\tBasepwm: ");
  Serial.print(basePwm);
 }

  /*  Function Name       : M_TurnTillPressed()
      Input               : Required angle(req_ang) for which function is to be executed and  Proportionality constants(KP_Orient, KI_Angle)
      Output              : Bot orients as per given required angle(req_angle).
      Logic               : Bot orients at desired or required angle from initial position, all motors are given equal pwm until it acquires desired angle.
      Example Call        : M_TurnTillPressed(1);
  */
  void Manual :: TTP_Man(int dir)
  {
    if (dir == 1)
    {
      dirW1 = 1;
      dirW2 = 0;
      dirW3 = 1;
    }
    else if (dir == 0)
    {
      dirW1 = 0;
      dirW2 = 1;
      dirW3 = 0;
    }

    Motor M1(34, dirW1, 9);
    Motor M2(28, dirW2, 6);
    Motor M3(30, dirW3, 7);

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

    M1.SetDirection();
    M2.SetDirection();
    M3.SetDirection();


    M1.SetSpeed(pwmm_ori1);
    M2.SetSpeed(pwmm_ori2);
    M3.SetSpeed(pwmm_ori3);
  }

  /*  Function Name       : M_Turn()
      Input               : Required angle(req_ang) for which function is to be executed and  Proportionality constants(KP_Orient, KI_Angle)
      Output              : Bot orients as per given required angle(req_angle).
      Logic               : Bot orients at desired or required angle from initial position, all motors are given equal pwm until it acquires desired angle.
      Example Call        : M_Turn(KP_Orient, KI_Angle, 338.00);
  */
  void Manual :: TurnMan(float KP_Orient, float KI_Angle, float req_angle, int dir)
  {
    error_ang = 5;
    while (rate_change != 0 || abs(error_ang) > 2)
    {
      // read from port 1, send to port 0:
      Yaw = V.readMpu();

      final_ang  = pow(-1, !dir) * req_angle + Shifted_Yaw ;
      final_ang += ((final_ang < - 180) - (final_ang > 180)) * 360 ;

      error_ang = final_ang - Yaw ;
      error_ang += ((error_ang < -180) - (error_ang > 180)) * 360 ;

      rate_change = abs(error_ang) - abs(prev_error);

      if (dir)
      {
        dirW1 = 1;
        dirW2 = 0;
        dirW3 = 1;
      }
      else
      {
        dirW1 = 0;
        dirW2 = 1;
        dirW3 = 0;
      }

      error_sum_ori = error_sum_ori + error_ang;

      if ((rate_change) > 0)
      {
        dirW1 = !(dirW1);
        dirW2 = !(dirW2);
        dirW3 = !(dirW3);
      }

      Motor M1(34, dirW1, 9);
      Motor M2(28, dirW2, 6);
      Motor M3(30, dirW3, 7);

      pwmm_ori = abs(error_ang) * KP_Orient;

      if (abs(error_ang) < 20)
      {
        pwmm_ori += (KI_Angle * error_sum_ori);
      }

      if (pwmm_ori > (Maxpwm / 3))
        pwmm_ori = (Maxpwm / 3);


      /********************************************* SERIAL PRINTING DATA ***************************************************/

      Serial.print("\tYaw: ");
      Serial.print(Yaw);
      Serial.print("\tError: ");
      Serial.print(error_ang);
      Serial.print("\tFinal: ");
      Serial.print(final_ang);
      Serial.print("\tKp:  ");
      Serial.print(KP_Orient);
      Serial.print("\tPWM:  ");
      Serial.print(pwmm_ori);
      Serial.print("\trate:  ");
      Serial.print(rate_change);
      Serial.print("\tprevious:  ");
      Serial.print(prev_error);
      Serial.print("\tdir:  ");
      Serial.println(dir);


      pwmm_ori1 = pwmm_ori;
      pwmm_ori2 = pwmm_ori;
      pwmm_ori3 = pwmm_ori;

      M1.SetDirection();
      M2.SetDirection();
      M3.SetDirection();

      M1.SetSpeed(pwmm_ori1);
      M2.SetSpeed(pwmm_ori2);
      M3.SetSpeed(pwmm_ori3);

      prev_error = error_ang;
    }
    M1.SetSpeed(0);
    M2.SetSpeed(0);
    M3.SetSpeed(0);

  }


#endif
