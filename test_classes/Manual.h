#include "Encoder.h"
#include "Mpu.h"
class Manual
{
private:
    /* data */
    /*** M_Forward() Function Variables ***/
float error_forward;                         //Variable to store the value of the X encoder as error.
#define Kp_encoder_forward 0.0              //Proportionality constant for the lateral error
#define Kp_strm2_forward  0.48                //Proportionality constant for the angular error for motor 2
#define Kp_strm3_forward  0.48                //Proportionality constant for the angular error for motor 3

float error_encoder_forward;
float pwm_encoder_forward;
float error_sum_forward;

/************ PWM Values ************/
#define Maxpwm 150.00
#define basePwm 100
int pwmm1, pwmm2, pwmm3;

int Yaw = 0;                                 //Variable to store the resolved value of yaw from -180 to 180
float Shifted_Yaw = 0;
public:
    Manual(/* args */);
    void forwardManY(kp_strm2_forward, kp_strm3_forward, kp_encoder_forward);
    void backwardManY();
};

Manual::Manual(/* args */)
{
}

void Manual :: forwardManY(kp_strm2_forward, kp_strm3_forward, kp_encoder_forward)
{
    Encoder X(21,50);
    Mpu V();

    Yaw = V.readMpu();
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

    digitalWrite(dir1, HIGH);
    digitalWrite(dir2, HIGH);
    digitalWrite(dir3, HIGH);

    analogWrite(pwm1, 0);
    analogWrite(pwm2, abs(pwmm2));
    analogWrite(pwm3, abs(pwmm3));

    /********************************************* SERIAL PRINTING DATA ***************************************************/
     
        Serial.print("yaw: ");
        Serial.print(yaw);
        Serial.print("\tYaw: ");
        Serial.print(Yaw);
        Serial.print("\tError: ");
        Serial.print(error_forward);
        Serial.print("\tError encoder: ");
        Serial.print(error_encoder_forward);
        Serial.print("\tencodervalue2 :      ");
        Serial.print(encodervalue1);
        Serial.print("\tPWM:  ");
        Serial.print(pwmm1);
        Serial.print("   ");
        Serial.print(pwmm2);
        Serial.print("   ");
        Serial.println(pwmm3);
    
}

void ManLocoY :: backwardManY()
{
    
}