#ifndef _Autonomus_H_
#define _Autonomous_H_

#include "Encoder.h"
#include "Mpu.h"
#include "Motor.h"

class Autonomous
{
    private :

    public :
    
/********** Direction pins **********/
#define dir1 34                                                                           //Direction pin for motor1
#define dir3 30                                                                           //Direction pin for motor3
#define dir2 28                                                                           //Direction pin for motor2
/********** Direction pins **********/

/************* PWM pins *************/
#define pwm1 9                                                                            //PWM pin for motor1
#define pwm3 7                                                                            //PWM pin for motor3
#define pwm2 6                                                                            //PWM pin for motor2
/************* PWM pins *************/

/************ PWM Values ************/
#define Maxpwm 200.00
#define Maxpwm1 180.00
int a_basePwm;
int pwmm1, pwmm2, pwmm3;
/************ PWM Values ************/

/********* Values of Yaw from MPU6050 *********/
int8_t yaw = 0;                                                                           // yaw values from MPU6050 using Serial Communications
float Yaw = 0, Shifted_Yaw = 0, Yaw_ref = 0;
int final_ang = 0,correction_angle = 0;
//Variable to store the resolved value of yaw from -180 to 180
/********* Values of Yaw from MPU6050 *********/

/***************************************************************************************************************************/
/********************************************** End of Common Variables ****************************************************/
/***************************************************************************************************************************/

/*****************************************************************************************************************************************/
/********************************************* Start of Variables From A_2W_Loc.h File ***************************************************/
/*****************************************************************************************************************************************/

/******* Variables for the encoder *******/
volatile int encodervalue1, encodervalue2;                                                // Count of pulses from encoder 1 and encoder 2
volatile int a, b, c, d;
/******* Variables for the encoder *******/

/************ Encoder Pins ************/
#define encoderPin1 2                                                                     //Interupt pin (A channel) for encoder1
#define encoderPin2 21                                                                    //Interrupt pin (A channel) for encoder2
#define comparePinA 50                                                                    // comparepin B channel for encoder1
#define comparePinB 52                                                                    //comparepin B channel for encoder2
/************ Encoder Pins ************/

/*** A_Forward() Function Variables ***/

/*float a_kp_dist_forward;                                                                  //Kp is given in proportion to the maxpwm and max error possible
float a_kp_encoder_forward;                                                               //Proportionality constant for the lateral error
float a_ki_dist_forward;                                                                  //Ki for distance
float a_kp_strm2_forward;                                                                 //Proportionality constant for the angular error for motor 2
float a_kp_strm3_forward;                                                                 //Proportionality constant for the angular error for motor 3
float a_kd2_dist_forward;
float a_kd3_dist_forward;
*/

float a_error_forward;                                                                    //Variable to store the value of the MPU as error
float a_error_encoder_forward;                                                            //Lateral shift error in forward direction
float a_pwm_encoder_forward2, a_pwm_encoder_forward3;                                     //PWM due to lateral shift in forward direction
float a_error_sum_forward = 0;                                                                //Adding previous errors for implementing Ki                                                                           
float a_current_forward, a_errorDist_forward;                                               //Current values of encoder and distance error in forward direction.                                                
float a_prev_error_forward;
float a_distanceCovered_forward;                                                          //Distance Covered in forward
float a_requiredDistance_forward;                                                         //Desired distance in forward 
float a_rateChange_forward = 0;
/*** A_Forward() Function Variables ***/

/*** A_Backward() Function Variables ***/
/*
float a_kp_dist_back;                                                                     //Kp is given in proportion to the maxpwm and max error possible
float a_ki_dist_back;                                                                     //Ki for distance
float a_kp_strm2_back;                                                                    //Proportionality constant for the angular error for motor 2
float a_kp_strm3_back;                                                                    //Proportionality constant for the angular error for motor 3
*/

float a_error_back;                                                                         //Variable to store the value of the MPU as error.
float a_error_encoder_back;                                                                 //Lateral shift error in back direction
float a_pwm_encoder_back;                                                                   //PWM due to lateral shift in back direction
float a_error_sum_back;                                                                     //Adding previous errors for implementing Ki                                             
float a_current_back, a_errorDist_back;                                                       //Current values of encoder and distance error in back direction.
float a_distanceCovered_back;                                                               //Distance Covered in back
float a_requiredDistance_back;                                                              //Desired distance in back

/*** A_Backward() Function Variables ***/

/*****************************************************************************************************************************************/
/********************************************** End of Variables From A_2W_Loc.h File ****************************************************/
/*****************************************************************************************************************************************/

/*****************************************************************************************************************************************/
/********************************************* Start of Variables From A_3W_Loc.h File ***************************************************/
/*****************************************************************************************************************************************/

/***** A_Left() Function Variables *****/
/*
float a_kp_dist_left;
float a_kp1_encoder_left,a_kp2_encoder_left,a_kp3_encoder_left;                                                                    //Kp is given in proportion to the maxpwm and max error possible
float a_kp_strm1_left;                                                                      //Proportionality constant for the angular error for motor 1
float a_kp_strm2_left;                                                                      //Proportionality constant for the angular error for motor 2
float a_kp_strm3_left;                                                                      //Proportionality constant for the angular error for motor 3
float a_ki_dist_left;                                                                       //Ki for distance
float a_kd1_dist_left;
float a_kd2_dist_left;
float a_kd3_dist_left;
*/
float a_errorDist_left;
float a_error_left;
float a_current_left, errorDist_left;                                                       //Current values of encoder and distance error in left direction.  
float a_prev_error_left;
float a_distanceCovered_left;                                                               //Distance Covered in left
float a_requiredDistance_left;                                                              //Desired distance in left
float a_error_encoder_left = 0;                                                                 //Lateral shift error in left direction
float a_pwm_encoder_left1,a_pwm_encoder_left2,a_pwm_encoder_left3;                                                                   //PWM due to lateral shift in left direction
float a_error_sum_left = 0;                                                                     //Adding previous errors for implementing Ki
float a_rateChange_left = 0;

/***** A_Left() Function Variables *****/

/***** A_Right() Function Variables *****/

float a_error_right;                                                                        //Variable to store the value of the MPU as error.
float a_kp_dist_right;
float a_kp_encoder_right;                                                                   //Kp is given in proportion to the maxpwm and max error possible
float a_kp_strm1_right;                                                                     //Proportionality constant for the angular error for motor 1
float a_kp_strm2_right;                                                                     //Proportionality constant for the angular error for motor 2
float a_kp_strm3_right;                                                                     //Proportionality constant for the angular error for motor 3
float a_ki_dist_right;                                                                      //Ki for distance
float a_kd1_dist_right;
float a_kd2_dist_right;
float a_kd3_dist_right;

float a_errorDist_right;
float a_current_right, errorDist_right;                                                     //Current values of encoder and distance error in right direction. 
float a_distanceCovered_right;                                                              //Distance Covered in right
float a_prev_error_right;
float a_requiredDistance_right;                                                             //Desired distance in right
float a_error_encoder_right;                                                                //Lateral shift error in right direction
float a_pwm_encoder_right;                                                                  //PWM due to lateral shift in right direction
float a_error_sum_right;                                                                    //Adding previous errors for implementing Ki
float a_rateChange_right = 0;

/***** A_Right() Function Variables *****/

/*****************************************************************************************************************************************/
/********************************************** End of Variables From A_3W_Loc.h File ****************************************************/
/*****************************************************************************************************************************************/

void forwardAutoY(float a_requiredDistance_forward, float a_kp_strm2_forward, float a_kp_strm3_forward, float a_kp_dist_forward, float a_kp2_encoder_forward,  float a_kp3_encoder_forward, float a_ki_dist_forward, float a_kd2_dist_forward, float a_kd3_dist_forward);
void backwardAutoY();
void rightAutoX();
void leftAutoX();   

};

#endif