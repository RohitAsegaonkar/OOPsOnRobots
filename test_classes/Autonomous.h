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

    if (a_errorDist_forward < ((a_requiredDistance_forward / 10.00) + 10))                    //Increasing the value of kp for angular deviation for motor 2, 10 cm before applying D so that it corrects itself
    {
      a_kp_strm2_forward += 0.67;
    }

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


void backwardAutoY(float a_requiredDistance_back, float a_kp_strm2_back, float a_kp_strm3_back, float a_kp_dist_back, float a_kp_encoder_back, float a_ki_dist_back)
{
  
  // encodervalue1 = 0;                                                                            //Shifting the origin by initializing both the encoder values to zero
  // encodervalue2 = 0;
  while (a_distanceCovered_back < a_requiredDistance_back)                                      //Execute the function till required distance is not reached
  {
    AutoYaw = _AutoMpu.readMpu(2);
    a_error_back = AutoYaw - AutoShifted_Yaw;                                                 //Calculate the angular shift of the bot. Yaw_ref is the reference yaw value from the previous function
    a_current_back = abs(_AutoY.getEncoderValueY());                                                        //Storing the value of the y encoder

    a_distanceCovered_back = a_current_back * 0.05236;                                          //Multiplying the value of the encoder by the circumference of the dummy wheel

    a_errorDist_back = a_requiredDistance_back - a_distanceCovered_back;                        //Calculating the error in distance

    a_error_sum_back = a_errorDist_back + a_error_sum_back;                                     //Calculating the sum of the errors

    a_basePwm = abs(a_errorDist_back) * a_kp_dist_back;                                         //Calculating the basepwm in proportion with the error

    a_error_encoder_back =  _AutoX.getEncoderValueX();                                                      //Error for locomotion in y direction is given by the x encoder
    a_pwm_encoder_back = a_kp_encoder_back * (a_error_encoder_back);                            //Calculating the pwm error

    a_pwmm2 = a_basePwm - a_kp_strm2_back * (a_error_back) - a_pwm_encoder_back + a_ki_dist_back * (a_error_sum_back);     //Calculating the pwm for motor 2 according to the equations of velocities
    a_pwmm3 = a_basePwm  + a_kp_strm3_back * (a_error_back) + a_pwm_encoder_back + a_ki_dist_back * (a_error_sum_back);    //Calculating the pwm for motor 3 according to the equations of velocities

    if (a_pwmm2 > Maxpwm)                                                                         //The pwm should not exceed the desired maximum pwm
      a_pwmm2 = Maxpwm;

    if (a_pwmm3 > Maxpwm)
      a_pwmm3 = Maxpwm;

     _AutoM1.SetDirection(0);
     _AutoM2.SetDirection(0);
     _AutoM3.SetDirection(0);
   
     _AutoM1.SetSpeed(0);
     _AutoM2.SetSpeed(abs(a_pwmm2));
     _AutoM3.SetSpeed(abs(a_pwmm3));

    /********************************************* SERIAL PRINTING DATA ***************************************************/
    Serial.print("yaw: ");
      Serial.print(AutoYaw);
      Serial.print("\tError: ");
      Serial.print(a_error_back);
      Serial.print("\tError encoder: ");
      Serial.print(a_error_encoder_back);
      Serial.print("\tencodervaluex :      ");
      Serial.print(_AutoX.getEncoderValueX());
      Serial.print("\tdistance covered :      ");
      Serial.print(a_distanceCovered_back);
      Serial.print("\tPWM:  ");
      Serial.print(a_pwmm1);
      Serial.print("   ");
      Serial.print(a_pwmm2);
      Serial.print("   ");
      Serial.println(a_pwmm3);
  }
  if (a_distanceCovered_back >= a_requiredDistance_back)                                      // Stoping the bot after the required distance is reached
{
     _AutoM1.SetSpeed(0);
     _AutoM2.SetSpeed(0);
     _AutoM3.SetSpeed(0);

  delay(100);
}

  //Setting the reference value for the next function that is called
  a_distanceCovered_back = 0;                                                                   //Flushing the value of the variable

}

void rightAutoX(float a_requiredDistance_right, float a_kp_strm1_right, float a_kp_strm2_right, float a_kp_strm3_right, float a_kp_dist_right, float a_kp_encoder_right, float a_ki_dist_right, float a_kd1_dist_right, float a_kd2_dist_right, float a_kd3_dist_right)
{
  
  // encodervalue1 = 0;                                                                          //Shifting the origin by initializing both the encoder values to zero
  // encodervalue2 = 0;          

  a_prev_error_right = a_requiredDistance_right; 
  while (a_distanceCovered_right < a_requiredDistance_right)                                  //Execute the function till required distance is not reached
  { 
    AutoYaw = _AutoMpu.readMpu(2);
    a_error_right = AutoYaw - AutoShifted_Yaw;                                                 //Calculate the angular shift of the bot. Yaw_ref is the reference yaw value from the previous function
    a_current_right = abs(_AutoX.getEncoderValueX());                                                        //Storing the value of the y encoder
    a_distanceCovered_right = a_current_right * 0.0555;                                      //Multiplying the value of the encoder by the circumference of the dummy wheel
    a_errorDist_right = a_requiredDistance_right - a_distanceCovered_right;                   //Calculating the error in distance
    a_error_sum_right = a_errorDist_right + a_error_sum_right;                                //Calculating the sum of the errors
    a_basePwm = abs(a_errorDist_right) * a_kp_dist_right;                                       //Calculating the basepwm in proportion with the error

    a_error_encoder_right = _AutoY.getEncoderValueY();                                                    //Error for locomotion in X direction is given by the y encoder
    a_pwm_encoder_right = a_kp_encoder_right * (a_error_encoder_right);                         //Calculating the pwm error

    a_pwmm1 =  a_basePwm + a_kp_strm1_right * (a_error_right) + a_pwm_encoder_right - 10 ;          //Calculating the pwm for motor 1 according to the equations of velocities
    a_pwmm2 = (a_basePwm - a_kp_strm2_right * (a_error_right) - a_pwm_encoder_right) / 2 + 5 ;   //Calculating the pwm for motor 2 according to the equations of velocities
    a_pwmm3 = (a_basePwm - a_kp_strm3_right * (a_error_right) - a_pwm_encoder_right) / 2 + 5 ;   //Calculating the pwm for motor 3 according to the equations of velocities


    a_rateChange_right = a_prev_error_right - a_errorDist_right;  

    if (a_errorDist_right < 25)                                                               //Implementing ki if error is less than 20
    {
      a_pwmm1 = a_pwmm1 + a_ki_dist_right * (a_error_sum_right);
      a_pwmm2 = a_pwmm2 + (a_ki_dist_right * (a_error_sum_right)) / 2;
      a_pwmm3 = a_pwmm3 + (a_ki_dist_right * (a_error_sum_right)) / 2;
    }

    
    //If Error is less than One Ninth of the Total Distance than apply D
      if (a_errorDist_right < (a_requiredDistance_right / 5.00))
      {
        a_pwmm1 -= (a_kd1_dist_right * a_rateChange_right);
        a_pwmm2 -= (a_kd2_dist_right * a_rateChange_right);
        a_pwmm3 -= (a_kd3_dist_right * a_rateChange_right);
        
        
      }

     //If PWM of any of the wheel is less than zero then make it zero
      if(a_pwmm1 < 0 || a_pwmm2 < 0 || a_pwmm3 < 0)
      {
        a_pwmm1 = 0;
        a_pwmm2 = 0;
        a_pwmm3 = 0;
      }

    if (a_pwmm1 > Maxpwm)                                                                       //The pwm should not exceed the desired maximum pwm
      a_pwmm1 = Maxpwm;

    if (a_pwmm2 > Maxpwm / 2)
      a_pwmm2 = Maxpwm / 2;

    if (a_pwmm3 > Maxpwm / 2)
      a_pwmm3 = Maxpwm / 2;


     _AutoM1.SetDirection(0);
     _AutoM2.SetDirection(0);
     _AutoM3.SetDirection(1);
   
     _AutoM1.SetSpeed(abs(a_pwmm1));
     _AutoM2.SetSpeed(abs(a_pwmm2));
     _AutoM3.SetSpeed(abs(a_pwmm3));
  
    /********************************************* SERIAL PRINTING DATA ***************************************************/
    
      
      Serial.print("\tYaw: ");
      Serial.print(AutoYaw);
      Serial.print("\tError: ");
      Serial.print(a_error_right);
      Serial.print("\tError encoder: ");
      Serial.print(a_error_encoder_right);
      Serial.print("\tencodervalueX:      ");
      Serial.print(_AutoX.getEncoderValueX());
      Serial.print("\tdistance covered :      ");
      Serial.print(a_distanceCovered_right);
      Serial.print("\tPWM:  ");
      Serial.print(a_pwmm1);
      Serial.print("   ");
      Serial.print(a_pwmm2);
      Serial.print("   ");
      Serial.println(a_pwmm3);
      Serial.print("\tBasepwm: ");
      Serial.print(a_basePwm);
    
    a_prev_error_right = a_errorDist_right;
  }
  if (a_distanceCovered_right >= a_requiredDistance_right)                                    // Stoping the bot after the required distance is reached
  {       
     _AutoM1.SetSpeed(0);
     _AutoM2.SetSpeed(0);
     _AutoM3.SetSpeed(0);
    delay(100);
  }       
  //Setting the reference value for the next function that is called        
  a_distanceCovered_right = 0;                                                                //Flushing the value of the variable

}

void leftAutoX(float a_requiredDistance_left, float a_kpstrm1_left, float a_kpstrm2_left, float a_kpstrm3_left, float a_kpdist_left, float a_kp1encoder_left, float a_kp2encoder_left, float a_kp3encoder_left, float a_kidist_left , float a_kd1_dist_left, float a_kd2_dist_left, float a_kd3_dist_left)
{
  
  // encodervalue1 = 0;                                                                          //Shifting the origin by initializing both the encoder values to zero
  // encodervalue2 = 0;            
  
  a_prev_error_left = a_requiredDistance_left;            
  
  while (a_distanceCovered_left < a_requiredDistance_left)                                    //Execute the function till required distance is not reached
  {           
    AutoYaw = _AutoMpu.readMpu(2);
    a_error_left = AutoYaw - AutoShifted_Yaw;                                                 //Calculate the angular shift of the bot. Yaw_ref is the reference yaw value from the previous function
    a_current_left = abs(_AutoX.getEncoderValueX());                                                        //Storing the value of the y encoder

    a_distanceCovered_left = a_current_left * 0.05236;                                        //Multiplying the value of the encoder by the circumference of the dummy wheel
    
    a_errorDist_left = a_requiredDistance_left - a_distanceCovered_left;                      //Calculating the error in distance
    a_error_sum_left = a_errorDist_left + a_error_sum_left;                                   //Calculating the sum of the errors
    
    a_basePwm = abs(a_errorDist_left) * a_kpdist_left;                                        //Calculating the basepwm in proportion with the error

    a_error_encoder_left = _AutoY.getEncoderValueY();;                                                     //Error for locomotion in X direction is given by the y encoder
    a_pwm_encoder_left1 = a_kp1encoder_left * (a_error_encoder_left);                         //Calculating the pwm error
    a_pwm_encoder_left2 = a_kp2encoder_left * (a_error_encoder_left);                         //Calculating the pwm error
    a_pwm_encoder_left3 = a_kp3encoder_left * (a_error_encoder_left);           

    a_rateChange_left = a_prev_error_left - a_errorDist_left;           

    a_pwmm1 = (a_basePwm  - a_kpstrm1_left * (a_error_left) + a_pwm_encoder_left1) - 20;        //Calculating the pwm for motor 1 according to the equations of velocities   offset for 400: -20
    a_pwmm2 = (a_basePwm  + a_kpstrm2_left * (a_error_left) - a_pwm_encoder_left2) / 2 + 15;    //Calculating the pwm for motor 2 according to the equations of velocities   offset for 400: +20
    a_pwmm3 = (a_basePwm  + a_kpstrm3_left * (a_error_left) - a_pwm_encoder_left3) / 2 ;        //Calculating the pwm for motor 3 according to the equations of velocities   

    //Implementing ki if error is less than 20
      if (a_errorDist_left < (a_requiredDistance_left/2.22))                                  
      {
        a_pwmm1 = a_pwmm1 + a_kidist_left * (a_error_sum_left) + 30;                                     //offset for 400: +30      
        a_pwmm2 = a_pwmm2 + (a_kidist_left * (a_error_sum_left)) / 2 - 10;                               //offset for 400: -10      
        a_pwmm3 = a_pwmm3 + (a_kidist_left * (a_error_sum_left)) / 2;
      }

    //If Error is less than One Ninth of the Total Distance than apply D
      if (a_errorDist_left < (a_requiredDistance_left / 10.00))
      {
        a_pwmm1 -= (a_kd1_dist_left * a_rateChange_left);
        a_pwmm2 -= (a_kd2_dist_left * a_rateChange_left);
        a_pwmm3 -= (a_kd3_dist_left * a_rateChange_left);
        
      }

    //If PWM of any of the wheel is less than zero then make it zero
      if(a_pwmm1 < 0 )
      {
        a_pwmm1 = 0;
      }
      if(a_pwmm2 < 0 )
      {
        a_pwmm2 = 0;
      }
      if(a_pwmm3 < 0 )
      {
        a_pwmm3 = 0;
      }        

    if (a_pwmm1 > Maxpwm)                                                                           //The pwm should not exceed the desired maximum pwm
      a_pwmm1 = Maxpwm;              
              
    if (a_pwmm2 > Maxpwm / 2)              
      a_pwmm2 = Maxpwm / 2;              
              
    if (a_pwmm3 > Maxpwm / 2)              
      a_pwmm3 = Maxpwm / 2;              
                         
     _AutoM1.SetDirection(1);
     _AutoM2.SetDirection(1);
     _AutoM3.SetDirection(0);
   
      _AutoM1.SetSpeed(abs(a_pwmm1));
      _AutoM2.SetSpeed(abs(a_pwmm2));
      _AutoM3.SetSpeed(abs(a_pwmm3));

    /********************************************* SERIAL PRINTING DATA ***************************************************/
    /*

      Serial.print("\tYaw: ");
      Serial.print(AutoYaw);
      Serial.print("\tError: ");
      Serial.print(a_error_left);
      Serial.print("\tError encoder: ");
      Serial.print(a_error_encoder_left);
      Serial.print("\tencodervaluey :      ");
      Serial.print(_AutoY.getEncoderValueY()a_);
      Serial.print("\tdistance covered :      ");
      Serial.print(a_distanceCovered_left);
      Serial.print("\tPWM:  ");
      Serial.print(a_pwmm1);
      Serial.print("   ");
      Serial.print(a_pwmm2);
      Serial.print("   ");
      Serial.println(a_pwmm3);
      Serial.print("\tBasepwm: ");
      Serial.print(a_basePwm);
      */
      a_prev_error_left = a_errorDist_left;
    
  }
  if (a_distanceCovered_left >= a_requiredDistance_left)                                            // Stoping the bot after the required distance is reached
  {
     _AutoM1.SetSpeed(0);
     _AutoM2.SetSpeed(0);
     _AutoM3.SetSpeed(0);
    delay(100);
  }
  //Setting the reference value for the next function that is called
  a_distanceCovered_left = 0;                                                                       //Flushing the value of the variable

}

};

#endif
