/* Filename                 : 
 * Name of Class            :
 * Name of Methods          :
 * Name of Member Variables :
 *
 */

class Motor
{
    private:
    int _DirPin;
    bool _Dir;
    int _PWM_Pin
    int _PWM;

    public:
    Motor(int _DirPin, bool _Dir, int _PWM_Pin);
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
    
}