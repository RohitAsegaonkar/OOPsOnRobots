/* Filename                 : Motor.h
 * Name of Class            : Motor
 * Name of Methods          : SetDirection(),ToggleDirction(),SetSpeed();
 * Name of Member Variables :
 *
 */

#ifndef _Motor_H_
#define _Motor_H_

class Motor
{
    private:
    int _DirPin;
    bool _Dir;
    int _PWM_Pin;
    int _PWM;

    

    public:

    Motor()
    {

    }
    
    Motor(int _DirPin_, bool _Dir_, int _PWM_Pin_)
    {
        _DirPin = _DirPin_;
        _Dir = _Dir_;
        _PWM_Pin = _PWM_Pin_;
        
        pinMode(_DirPin,OUTPUT);
        pinMode(_PWM_Pin,OUTPUT);
    }

    void SetDirection();
    void SetDirection(bool _dir);
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

void Motor::SetDirection(bool _dir)
{
    digitalWrite(_DirPin, _dir);
}

void Motor::SetSpeed(int _PWM)
{
    analogWrite(_PWM_Pin,_PWM);
    //Serial.print(_PWM_Pin);
}
#endif
