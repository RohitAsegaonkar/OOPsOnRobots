# OOPsOnRobots
Classes and objects implementation.

## Class Piston
Class piston is used to represent a piston.<br>

#### Member Variables:
```cpp
 -  int Extension_Pin. 
 -  int Retraction_Pin.
 ```

#### Member Functions:
```cpp
 - void Extend().
 - void Retract(). 
 ```

### Creating a object of class piston
Meaning of parameters:
 - **Extension_Pin**    - Pin which is given high to extend the piston.
 - **Retraction_Pin**   - Pin which is given high to retract the piston.

```cpp
Piston P1(int Extension_Pin, int Retraction_Pin);
```

### Invoking Member Functions
```cpp
Piston P1(int Extension_Pin, int Retraction_Pin);

P1.Extend();      //Extending the Piston
P1.Retract();     //Retracting the Piston

```

## Class Encoder
Class Encoder is used to represent a Encoder.<br>

#### Member Variables:
```cpp
- volatile int encodervalue;
- volatile int a, b;
- int encoderPin , comparePin; 
 ```

 #### Member Functions:
```cpp
- void updateEncoder().
 ```
 ### Creating a object of class encoder
 Meaning of parameters:
 - **encodervalue**  - Variable which stores the Encoder value.
 - **encoderPin** - Encoder Pin Number connected to Controller.
 - **comparePin** - Compare Pin Number connected to Controller.

 ```cpp
Encoder X(int EncoderPin, int ComparePin);
```

### Invoking Member Functions
```cpp
Encoder X(int EncoderPin, int ComparePin);

X.updateEncoder();     //Updating Encoder Values

```

## Class Mpu
Class Mpu is used to represent the inertial measurement unit MPU6050.<br>

#### Member Variables:
```cpp
```

#### Member Functions:
```cpp
- int readMpu(int s).
```
### Creating an object of class Mpu
Meaning of parameters:
* s - to select the serial port <br>

| s  | serial port |
|--- |-------------| 
| 1  | serial1     | 
| 2  | serial2     |
| 3  | serial3     |

```cpp 
Mpu V;
```
### Invoking Member Functions
```cpp
Mpu V;

V.readMpu(2);      //reads available data from serial port 2
```


