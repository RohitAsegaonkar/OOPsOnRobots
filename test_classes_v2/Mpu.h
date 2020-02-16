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
    float readMpu(int s);
    int8_t yaw = 0;
    float _Yaw;
};

 Mpu::Mpu(/* args */)
{
   yaw = 0;
   _Yaw = 0;
}

float Mpu::readMpu(int s)
{
    
                if (Serial2.available())
            {
                yaw = Serial2.read();
            }

            _Yaw = yaw * (180.00 / 127.00);

            return _Yaw;    

} 


#endif
