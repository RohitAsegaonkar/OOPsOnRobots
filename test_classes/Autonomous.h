#ifndef _Autonomus_H_
#define _Autonomous_H_

#include "Encoder.h"
#include "Mpu.h"
#include "Motor.h"

class Autonomous
{
    private :
    /******************************************* Creation of Objects ******************************************************/
    Encoder _AutoX;
    Encoder _AutoY;
    Mpu _AutoMpu;
    Motor _AutoM1, _AutoM2, _AutoM3;
    /**********************************************************************************************************************/
    
    /********************************************** MPU Variables *********************************************************/
    float AutoShifted_Yaw = 0;              //Variable to store the changed/shifted value of yaw as set point for next function
    float AutoYaw = 0;                      //Variable to store the resolved value of yaw from -180 to 180
    float Yaw_ref = 0;
    float final_ang = 0;                    //Variable to store the value of the final angle in the TurnMan function
    float correction_angle = 0;             //Variable to store the correction angle in th eself orient function
    /**********************************************************************************************************************/

    /*********************************************** PWM Values ***********************************************************/
    #define Maxpwm 150.00                   //Maximum pwm as constraint in the entire code
    int a_basePwm;
    int a_pwmm1, a_pwmm2, a_pwmm3;
    /**********************************************************************************************************************/

    /************************************* forwardManY() Function Variables ***********************************************/
    float a_error_forward = 0;                                //Variable to store the value of the MPU as error
    float a_error_encoder_forward = 0;                        //Lateral shift error in forward direction
    float a_pwm_encoder_forward2 = 0, a_pwm_encoder_forward3 = 0; //PWM due to lateral shift in forward direction
    float a_error_sum_forward = 0;                        //Adding previous errors for implementing Ki                                                                           
    float a_current_forward = 0, a_errorDist_forward = 0;         //Current values of encoder and distance error in forward direction.                                                
    float a_prev_error_forward = 0;
    float a_distanceCovered_forward = 0;       //Distance Covered in forward
    float a_requiredDistance_forward = 0;      //Desired distance in forward 
    float a_rateChange_forward = 0;
    /*** A_Forward() Function Variables ***/

    /**********************************************************************************************************************/

    /************************************** backwardManY() Function Variables *********************************************/
    float a_error_back;                                                                         //Variable to store the value of the MPU as error.
    float a_error_encoder_back;                                                                 //Lateral shift error in back direction
    float a_pwm_encoder_back;                                                                   //PWM due to lateral shift in back direction
    float a_error_sum_back;                                                                     //Adding previous errors for implementing Ki                                             
    float a_current_back, a_errorDist_back;                                                       //Current values of encoder and distance error in back direction.
    float a_distanceCovered_back;                                                               //Distance Covered in back
    float a_requiredDistance_back;                                                              //Desired distance in back

    /*** A_Backward() Function Variables ***/
    /**********************************************************************************************************************/

    /*************************************** leftManX() Function Variables ************************************************/
    float error_encoder_left;
    float error_sum_left;
    float a_error_left;
    float a_current_left; 
    float a_errorDist_left;                                                       //Current values of encoder and distance error in left direction.  
    float a_prev_error_left;
    float a_distanceCovered_left;                                                               //Distance Covered in left
    float a_requiredDistance_left;                                                              //Desired distance in left
    float a_error_encoder_left = 0;                                                                 //Lateral shift error in left direction
    float a_pwm_encoder_left1,a_pwm_encoder_left2,a_pwm_encoder_left3;                                                                   //PWM due to lateral shift in left direction
    float a_error_sum_left = 0;                                                                     //Adding previous errors for implementing Ki
    float a_rateChange_left = 0;
    /**********************************************************************************************************************/

    /****************************************** rightManX() Function Variables ********************************************/
    
    float a_error_right;                                                                        //Variable to store the value of the MPU as error.
    float a_current_right;
    float a_errorDist_right;                                                     //Current values of encoder and distance error in right direction. 
    float a_distanceCovered_right;                                                              //Distance Covered in right
    float a_prev_error_right;
    float a_requiredDistance_right;                                                             //Desired distance in right
    float a_error_encoder_right;                                                                //Lateral shift error in right direction
    float a_pwm_encoder_right;                                                                  //PWM due to lateral shift in right direction
    float a_error_sum_right;                                                                    //Adding previous errors for implementing Ki
    float a_rateChange_right = 0;
    /**********************************************************************************************************************/

    /***************************************** TurnMan() Function Variables ***********************************************/
    float req_angle = 0.0;
    float error_ang = 3;
    float error_sum_ori;
    float prev_error = 0;
    /**********************************************************************************************************************/


    public :

    Autonomous(Motor m1, Motor m2, Motor m3, Mpu v, Encoder x, Encoder y)
    {
      //Assigning the Motor Object
      _AutoM1 = m1;
      _AutoM2 = m2;
      _AutoM3 = m3;
      //Assigning the Encoder Object
      _AutoX = x;
      _AutoY = y;
      //Assigning the MPU object
      _AutoMpu = v;

      //_AutoY.info();
    }    


void forwardAutoY(float a_requiredDistance_forward, float a_kp_strm2_forward, float a_kp_strm3_forward, float a_kp_dist_forward, float a_kp2_encoder_forward,  float a_kp3_encoder_forward, float a_ki_dist_forward, float a_kd2_dist_forward, float a_kd3_dist_forward)
{
  //_AutoX.encodervalue = 0;                                                                          //Shifting the origin by initializing both the encoder values to zero
  //_AutoY.encodervalue = 0;
  a_prev_error_forward = a_requiredDistance_forward;

  while (a_distanceCovered_forward < a_requiredDistance_forward)                              //Execute the function till required distance is not reached
  {    
    AutoYaw = _AutoMpu.readMpu(2);
    a_error_forward = AutoYaw - AutoShifted_Yaw;                                                      //Calculate the angular shift of the bot. Yaw_ref is the reference yaw value from the previous function

    a_current_forward = abs(_AutoY.getEncoderValueY());                                                   //Storing the value of the y encoder

    a_distanceCovered_forward = a_current_forward * 0.05236;                                  //Multiplying the value of the encoder by the circumference of the dummy wheel

    a_errorDist_forward = a_requiredDistance_forward - a_distanceCovered_forward;             //Calculating the error in distance
    a_error_sum_forward = a_errorDist_forward + a_error_sum_forward;                          //Calculating the sum of the errors

    a_basePwm = abs(a_errorDist_forward) * a_kp_dist_forward;                                 //Calculating the basepwm in proportion with the error

    a_error_encoder_forward = _AutoX.getEncoderValueX();                                                  //Error for locomotion in y direction is given by the x encoder

    a_pwm_encoder_forward2 = a_kp2_encoder_forward * (a_error_encoder_forward);               //Calculating the pwm error due to Lateral Shift
    a_pwm_encoder_forward3 = a_kp3_encoder_forward * (a_error_encoder_forward);               //Calculating the pwm error due to Lateral Shift

    a_rateChange_forward = a_prev_error_forward - a_errorDist_forward;                        //Calculating the rate of change of error in distance

    a_pwmm2 = a_basePwm  + a_kp_strm2_forward * (a_error_forward) + a_pwm_encoder_forward2 ;    //Calculating the pwm for motor 2 according to the equations of velocities
    a_pwmm3 = a_basePwm  - a_kp_strm3_forward * (a_error_forward) - a_pwm_encoder_forward3 ;    //Calculating the pwm for motor 3 according to the equations of velocities

//    if (a_errorDist_forward < ((a_requiredDistance_forward / 10.00) + 10))                    //Increasing the value of kp for angular deviation for motor 2, 10 cm before applying D so that it corrects itself
//    {
//      a_kp_strm2_forward += 0.005;
//    }

    if (a_errorDist_forward < (a_requiredDistance_forward / 5.00))                          //If Error is less than One Ninth of the Total Distance than apply D
    {
      a_pwmm2 -= (a_kd2_dist_forward * a_rateChange_forward);
      a_pwmm3 -= (a_kd3_dist_forward * a_rateChange_forward);
      a_kp_strm3_forward += 0.1;
      a_kp_strm2_forward += 0.1;
      
    }

    if (a_pwmm2 < 0 )                                                                          //If PWM of any wheel becomes negative make it zero
      a_pwmm2 = 0;

    if (a_pwmm3 < 0)
      a_pwmm3 = 0;


    if (a_pwmm2 > Maxpwm)                                                                       //The pwm should not exceed the desired maximum pwm
      a_pwmm2 = Maxpwm;

    if (a_pwmm3 > Maxpwm)
      a_pwmm3 = Maxpwm;
    
     _AutoM1.SetDirection(0);
     _AutoM2.SetDirection(1);
     _AutoM3.SetDirection(1);
   
     _AutoM1.SetSpeed(0);
     _AutoM2.SetSpeed(abs(a_pwmm2));
     _AutoM3.SetSpeed(abs(a_pwmm3));


    /********************************************* SERIAL PRINTING DATA ***************************************************/

//    Serial.print("\tYaw: ");
//    Serial.print(AutoYaw);
//    Serial.print("\tError: ");
//    Serial.print(a_error_forward);
//    Serial.print("\tError encoder: ");
//    Serial.print(a_error_encoder_forward);
//    Serial.print(" encodervalueX :      ");
//    Serial.print(_AutoX.getEncoderValueX());
//      Serial.print(" \tencodervalueY:      ");
//    Serial.print(_AutoX.getEncoderValueY());
//    Serial.print("\tdistance covered :      ");
//    Serial.print(a_distanceCovered_forward);
//    Serial.print("\tPWM:  ");
//    Serial.print(a_pwmm1);
//    Serial.print("   ");
//    Serial.print(a_pwmm2);
//    Serial.print("   ");
//    Serial.println(a_pwmm3);

    a_prev_error_forward = a_errorDist_forward;                                               //Storing the value of the previoud error inorder to calculate the rate of change of error in distance
  }
  if (a_distanceCovered_forward >= a_requiredDistance_forward)                                  // Stoping the bot after the required distance is reached
  {
     _AutoM1.SetSpeed(0);
     _AutoM2.SetSpeed(0);
     _AutoM3.SetSpeed(0);

    delay(100);
  }
  //Setting the reference value for the next function that is called
  a_distanceCovered_forward = 0;                                                                //Flushing the value of the variable
}



void backwardAutoY();
void rightAutoX();
void leftAutoX();   

};

#endif
