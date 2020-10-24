/* Filename                 : Piston.h
 * Name of Class            : Piston
 * Name of Methods          : Extend(), Retract()
 * Name of Member Variables : Extend_Pin, Retract_Pin
 *
 */

#ifndef _Piston_H_
#define _Piston_H_

class Piston
{
    private:
    int Extend_Pin, Retract_Pin;

    public:
    Piston(int E_Pin, int R_pin)
    {
        Extend_Pin = E_Pin;
        Retract_Pin = R_pin;

        pinMode(Extend_Pin, OUTPUT);
        pinMode(Retract_Pin, OUTPUT);

        digitalWrite(Extend_Pin, LOW);
        digitalWrite(Retract_Pin, LOW);
    }

    void Extend();
    void Retract();

};

void Piston::Extend()
{
    digitalWrite(Extend_Pin, HIGH);
    digitalWrite(Retract_Pin, LOW);
}

void Piston::Retract()
{
    digitalWrite(Extend_Pin, LOW);
    digitalWrite(Retract_Pin, HIGH);
}

#endif