#ifndef _Encoder_H_
#define _Encoder_H_

class Encoder
{
    private:
    int pwm;
    volatile int a, b, c, d;
    int encoderPin , comparePin; 
    /******* Variables for the encoder *******/
                                            // Count of pulses from encoder 1 and encoder 2
    
    public:
    static int EncoderValueX;
    static int EncoderValueY;
    volatile int encodervalue = 0;         
    Encoder()
    {

    }
    Encoder(int EP1, int CP1)
    {
        encoderPin = EP1;
        comparePin = CP1;
        pinMode(encoderPin,INPUT_PULLUP);
        pinMode(comparePin,INPUT_PULLUP);
    }


/*  Function Name       : updateEncoder()
 *  Input               : No input required
 *  Output              : pulses of the x encoder stored in variable encodervalue1
 *  Logic               : The function will be called when the interrupt is triggered at the rising edge of channel A.
 *                        If the channel B is in phase with the channel A the count is incremented and decremented otherwise
 *  Example Call        : The function is called by the interrupt
 */
    void updateEncoderX()
    {
        a = digitalRead(encoderPin);
        b = digitalRead(comparePin);
        if (b == 1)
            encodervalue++;
        if (b == 0)
            encodervalue--;
        EncoderValueX = encodervalue;
    }   

    void updateEncoderX(int m)
    {
        encodervalue = m;
        EncoderValueX = encodervalue;
    }         



     void updateEncoderY()
    {
        c = digitalRead(encoderPin);
        d = digitalRead(comparePin);
        if (d == 1)
            encodervalue++;
        if (d == 0)
            encodervalue--;
        EncoderValueY = encodervalue;
    }      

    void updateEncoderY(int m)
    {
        encodervalue = m;
        EncoderValueY = encodervalue;
    }      


    void info(){
      Serial.print("a:");
      Serial.print(a);
      Serial.print("\t");
      Serial.print("b:");
      Serial.print(b);
      Serial.print("\t");
      Serial.print(encodervalue);
    }

     int getEncoderValueX()
    {  

//        Serial.print("encodervalue\t");
//        Serial.println(EncoderValue);
        return EncoderValueX;
    }

       int getEncoderValueY()
    {  
 
//        Serial.print("encodervalue\t");
//        Serial.println(EncoderValue);
        return EncoderValueY;
    } 

        void DebugEncoderX(int posPin,int negPin)
        {
            //for(int i = 0; i < 2000; i++)
            //{
                if(getEncoderValueX() < 0)
                {
                    analogWrite(posPin, map(getEncoderValueX(), 0, 600, 0, 255));
                    analogWrite(negPin, 0);
                }

                else if(getEncoderValueX() > 0)
                {
                    analogWrite(negPin ,map(getEncoderValueX(), -1, -600, 0, 255));
                    analogWrite(posPin, 0);
                }

                else 
                {
                    analogWrite(negPin, 0);
                    analogWrite(posPin, 0); 
                }
            
        }

         void DebugEncoderY(int posPin,int negPin)
        {
            //for(int i = 0; i < 2000; i++)
            //{
                if(getEncoderValueY() < 0)
                {
                    analogWrite(posPin, map(getEncoderValueY(), 0, 600, 0, 255));
                    analogWrite(negPin, 0);
                }
                else if(getEncoderValueY() > 0)
                {
                    analogWrite(negPin, map(getEncoderValueY(), -1, -600, 0, 255));
                    analogWrite(posPin, 0);
                }

                else 
                {
                    analogWrite(negPin, 0);
                    analogWrite(posPin, 0); 
                }
            }
        

};


static int Encoder::EncoderValueX = 0;
static int Encoder::EncoderValueY = 0;

#endif
