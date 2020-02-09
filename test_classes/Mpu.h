/* Filename                 : Mpu.h
 * Name of Class            : Mpu
 * Name of Methods          : readMpu()
 * Name of Member Variables : 
 *
 */

#ifndef _Mpu_H_
#define _Mpu_H_

class Mpu
{
private:
    /* data */
public:
    Mpu(/* args */);
    int readMpu(int s);
};

Mpu::Mpu(/* args */)
{
}

int Mpu::readMpu(int s=3)
{
    int8_t yaw;
    int _Yaw;

    switch (s) 
    {
        case 1:
        {
            if (Serial1.available())
            {
                yaw = Serial1.read();
            }

            _Yaw = yaw * (180.00 / 127.00);

            return _Yaw;    

            break;
        } 
            
        case 2:
        {
            if (Serial2.available())
            {
                yaw = Serial2.read();
            }

            _Yaw = yaw * (180.00 / 127.00);

            return _Yaw;    

            break;
        }        
        case 3:
        {
            if (Serial3.available())
            {
                yaw = Serial3.read();
            }

            _Yaw = yaw * (180.00 / 127.00);

            return _Yaw;    

            break;
        }

    }
}
#endif
