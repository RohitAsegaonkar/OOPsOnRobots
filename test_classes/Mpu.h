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

int Mpu::readMpu()
{
    int8_t yaw;
    int Yaw;
    if (Serial3.available())
    {
        yaw = Serial3.read();
    }
    Yaw = yaw * (180.00 / 127.00);
    return Yaw;
}