/* Filename                 : 
 * Name of Calss            :
 * Name of Methods          :
 * Name of Member Variables :
 *
 */

class Motor
{
    private:
    int _Pin_No;
    bool _Dir;
    int _PWM;

    public:
    Motor(int _Pin_No, bool _Dir);
    void Toggle_Diretion();
    void ChangeSpeed(int _PWM);
};