#ifndef _Manual_
#define _Manual_

#include "Encoder.h"
#include "Mpu.h"
#include "Motor.h"

class Manual
{
private:
    /*** forwardManY() Function Variables ***/
float error_forward;                                 //Variable to store the value of the X encoder as error.
#define Kp_encoder_forward 0.0                       //Proportionality constant for the lateral error
#define Kp_strm2_forward  0.48                       //Proportionality constant for the angular error for motor 2
#define p_strm3_forward  0.48                        //Proportionality constant for the angular error for motor 3

float error_encoder_forward;
float pwm_encoder_forward;
float error_sum_forward;

/*** backwardManY() Function Variables ***/
float error_back;
#define kp_encoder_back 0.068
#define kp_strm2_back 0.25
#define kp_strm3_back 0.25

float error_encoder_back;
float pwm_encoder_back;
float error_sum_back;

/***** leftManX() Function Variables *****/
float error_left;                                     //Variable to store the value of the X encoder as error.
#define kp_encoder_left 0.02
#define kp_strm1_left 0.4
#define kp_strm2_left 0.6
#define kp_strm3_left 0.6

float error_encoder_left;
float pwm_encoder_left;
float error_sum_left;

/***** rightManX() Function Variables *****/
float error_right;
#define kp_encoder_right 0.03
#define kp_strm1_right 0.4
#define kp_strm2_right 0.4
#define kp_strm3_right 0.45

float error_encoder_right;
float pwm_encoder_right;
float error_sum_right;

public:
    Manual(/* args */);
  //Boolean Variables to set direction of Wheels
    bool dirW1 = HIGH;
    bool dirW2 = HIGH;
    bool dirW3 = HIGH;
    int Yaw = 0;                                          //Variable to store the resolved value of yaw from -180 to 180
    float Shifted_Yaw = 0;
    /************ PWM Values ************/
    #define Maxpwm 150.00
    #define basePwm 100
    int pwmm1, pwmm2, pwmm3;
        /************************ Objects Created *********************************/
    Encoder X(21,50);
    Encoder Y(2,52);
    Mpu V();
    Motor M1(34,dirW1,9);
    Motor M2(28,dirW2,6);
    Motor M3(30,dirW3,7);
    void forwardManY(float kp_strm2_forward, float kp_strm3_forward, float kp_encoder_forward);
    void backwardManY(float KP_M2_Backward, float KP_M3_Backward, float KP_Enc_Backward);
    void leftManX(float KP_M1_Left, float KP_M2_Left, float KP_M3_Left, float KP_Enc_Left);
    void rightManX(float KP_M1_Right, float KP_M2_Right, float KP_M3_Right, float KP_Enc_Right);
};

Manual::Manual(/* args */)
{
}

/*
 *  Function Name       : forwardManY()
 *  Input               : Required distance(requiredDistance_forward) for which function is to be executed,Proportionality constants of Each Motors(kp_strm1_forward,KP_M2_Forward,KP_M3_Forward),
 *                        Encoders(KP_Enc_Forward) and Distance constant(kp_dist_forward,ki_dist_forward)
 *  Output              : Motor 2 and 3 will move with same pwm and Motor 1 will have zero PWM. Bot will move in forward Direction(along Motor 1).
 *  Logic               : By giving same PWM to Motors 2 and 3 and Motor 1 will have zero PWM,the bot will move in forward direction. MPU and two XY Encoders are used for FEEDBACK of System.
 *                        PID is implemented for precise movement.
 *  Example Call        : forwardManY(float kp_strm2_forward,float kp_strm3_forward,float kp_encoder_forward);
 */
void Manual :: forwardManY(float kp_strm2_forward,float kp_strm3_forward,float kp_encoder_forward)
{

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
 *  Function Name    : backwardManY()
 *  Input            : Required distance(requiredDistance_back) for which function is to be executed,Proportionality constants of Each Motors(kp_strm1_back,KP_M2_Backward,KP_M3_Backward),
 *                     Encoders(KP_Enc_Backward) and Distance constant(kp_dist_back,ki_dist_back)
 *  Output           : Motor 2 and 3 will move with same pwm and Motor 1 will have zero PWM. Bot will move in back Direction(along Motor 1 backward).
 *  Logic            : By giving same PWM to Motors 2 and 3 and Motor 1 will have zero PWM,the bot will move in backward direction. MPU and two XY Encoders are used for FEEDBACK of System.
 *                     PID is implemented for precise movement.
 *  Example Call     : backwardManY(float KP_M2_Backward, float KP_M3_Backward, float KP_Enc_Backward);
 */
void Manual :: backwardManY(float KP_M2_Backward, float KP_M3_Backward, float KP_Enc_Backward)
{


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

    M1.ToggleDirection();
    M2.ToggleDirection();
    M3.ToggleDirection();

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
 *  Function Name         : leftManX()
 *  Input                 : Required distance(requiredDistance_left) for which function is to be executed,Proportionality constants of Each Motors(KP_M1_Left,KP_M2_Left,KP_M3_Left),
 *                          Encoders(KP_Enc_Left) and Distance constant(kp_dist_left,ki_dist_left)
 *  Output                : Motor 2 and 3 will move with same pwm and Motor 1 will move with double the pwm of rest two Motors. Bot will move in Left Direction.
 *  Logic                 : By giving same PWM to Motors 2 and 3 and giving double PWM to Motor 1,the bot will move in left direction. MPU and two XY Encoders are used for FEEDBACK of System.
 *                          PID is implemented for precise movement.
 *  Example Call          : leftManX(float KP_M1_Left, float KP_M2_Left, float KP_M3_Left, float KP_Enc_Left);
 */
void Manual :: leftManX(float KP_M1_Left, float KP_M2_Left, float KP_M3_Left, float KP_Enc_Left)
{

    Yaw = V.readMpu();
    error_left = Yaw - Shifted_Yaw;                                                 //Calculate the angular shift of the bot. Yaw_ref is the reference yaw value from the previous function                       //Calculating the basepwm in proportion with the error

    error_encoder_left = Y.encodervalue;                                            //Error for locomotion in X direction is given by the y encoder
    pwm_encoder_left = KP_Enc_Left * (error_encoder_left);                          //Calculating the pwm error

    pwmm1 = basePwm - KP_M1_Left * (error_left)-pwm_encoder_left;                   //Calculating the pwm for motor 1 according to the equations of velocities
    pwmm2 = (basePwm + KP_M2_Left * (error_left) + pwm_encoder_left) / 2 + 10;      //Calculating the pwm for motor 2 according to the equations of velocities
    pwmm3 = (basePwm + KP_M3_Left * (error_left) + pwm_encoder_left) / 2 + 10;      //Calculating the pwm for motor 3 according to the equations of velocities

    if (pwmm1 > Maxpwm)                                                             //The pwm should not exceed the desired maximum pwm
        pwmm1 = Maxpwm;

    if (pwmm2 > Maxpwm)
        pwmm2 = Maxpwm / 2;

    if (pwmm3 > Maxpwm)
        pwmm3 = Maxpwm / 2;

    M1.SetDirection();
    M2.SetDirection();                                                              //Giving appropriate direction for the wheels according to left direction
    M3.ToggleDirection();
    

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
 *  Function Name        : rightManX()
 *  Input                : Required distance(requiredDistance_right) for which function is to be executed,Proportionality constants of Each Motors(KP_M1_Right,KP_M2_Right,KP_M3_Right),
 *                         Encoders(KP_Enc_Right) and Distance constant(kp_dist_right,ki_dist_right)
 *  Output               : Motor 2 and 3 will move with same pwm and Motor 1 will move with double the pwm of rest two Motors. Bot will move in Right Direction.
 *  Logic                : By giving same PWM to Motors 2 and 3 and giving double PWM to Motor 1,the bot will move in right direction. MPU and two XY Encoders are used for FEEDBACK of System.
 *                         PID is implemented for precise movement.
 *  Example Call         : rightManX(float KP_M1_Right, float KP_M2_Right, float KP_M3_Right, float KP_Enc_Right);
 */
void Manual :: rightManX(float KP_M1_Right, float KP_M2_Right, float KP_M3_Right, float KP_Enc_Right)
{

    Yaw = V.readMpu();

    error_encoder_right = Y.encodervalue;
    pwm_encoder_right = KP_Enc_Right * (error_encoder_right);

    pwmm1 = basePwm + KP_M1_Right * (error_right)-pwm_encoder_right;
    pwmm2 = (basePwm - KP_M2_Right * (error_right) + pwm_encoder_right) / 2 + 12;
    pwmm3 = (basePwm - KP_M3_Right * (error_right) + pwm_encoder_right) / 2 + 12;

    if (pwmm1 > Maxpwm)
        pwmm1 = Maxpwm;

    if (pwmm2 > Maxpwm)
        pwmm2 = Maxpwm / 2;

    if (pwmm3 > Maxpwm)
        pwmm3 = Maxpwm / 2;

    M1.ToggleDirection();
    M2.ToggleDirection();
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
#endif
