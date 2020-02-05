/* Filename                 : Motor.h
 * Name of Class            :Motor
 * Name of Methods          :SetDirection(),ToggleDirction(),SetSpeed();
 * Name of Member Variables :
 *
 */
#ifndef _MOTOR_H_
#define _MOTOR_H_

class Motor
{
    private:
    int _DirPin;
    bool _Dir;
    int _PWM_Pin;
    int _PWM;

    public:
     Motor(int DirPin, bool Dir, int PWM_Pin)
    {
        _DirPin = DirPin;
        _Dir = Dir;
        _PWM_Pin = PWM_Pin;
        pinMode(_DirPin,OUTPUT);
        pinMode(_PWM_Pin,OUTPUT);
    }

    void SetDirection();
    void ToggleDirection();
    void SetSpeed(int _PWM);

};

void Motor::ToggleDirection()
{
    _Dir = !(_Dir);
    digitalWrite(_DirPin,_Dir);

}

void Motor::SetDirection()
{
    digitalWrite(_DirPin,_Dir);
}

void Motor::SetSpeed(int _PWM)
{
    analogWrite(_PWM_Pin,_PWM);
    Serial.print("PIN :");
    Serial.print(_PWM_Pin);
    Serial.print("PWM :");
    Serial.println(_PWM);
}

#endif
