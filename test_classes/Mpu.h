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
    int8_t yaw = 0;
    float _Yaw;
    float readMpu(int s = 2);
    void DebugMpu(int pos, int neg);
};

 Mpu::Mpu(/* args */)
{
    yaw = 0;
    _Yaw = 0;
}

float Mpu::readMpu(int s = 2)
{
      

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

void Mpu::DebugMpu(int pos, int neg)
{
    for(int i=0;i<2000;i++){
    if(readMpu(2)<0){
    analogWrite(neg,(map(readMpu(2),-1,-180,0,255)));
    analogWrite(pos,0);
    }
    else if(readMpu(2)>0){
    analogWrite(pos,(map(readMpu(2),1,180,0,255)));
    analogWrite(neg,0);
    }
    else{
        analogWrite(pos,0);
        analogWrite(neg,0);
    }
    }

}

#endif
