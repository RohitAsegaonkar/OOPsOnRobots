class Mpu
{
private:
    /* data */
public:
    Mpu(/* args */);
    int readMpu();
};

Mpu::Mpu(/* args */)
{
}

int Mpu::readMpu(int s = 3)
{
    int8_t yaw;
    int Yaw;

    switch (s) 
    {
        case 1:
        {
            if (Serial1.available())
            {
                yaw = Serial1.read();
            }

            Yaw = yaw * (180.00 / 127.00);
            
            return Yaw;    

            break;
        } 
            
        case 2:
        {
            if (Serial2.available())
            {
                yaw = Serial2.read();
            }

            Yaw = yaw * (180.00 / 127.00);
            
            return Yaw;    

            break;
        }        
        case 3:
        {
            if (Serial3.available())
            {
                yaw = Serial3.read();
            }

            Yaw = yaw * (180.00 / 127.00);

            return Yaw;    

            break;
        }

    }
}
