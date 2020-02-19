# OOPsOnRobots

Classes and objects implementation.

## 1.Class Piston

Class piston is used to represent a piston.

### Member Variables

```cpp
 -  int Extension_Pin.
 -  int Retraction_Pin.
 ```

### Member Functions of Class Piston

```cpp
 - void Extend().
 - void Retract().
 ```

### Creating a object of Class Piston

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

## 2.Class Encoder

Class Encoder is used to represent a Encoder.

### Member Variables of Class Encoder

```cpp
- volatile int encodervalue;
- volatile int a, b;
- int encoderPin , comparePin;
 ```

### Member Functions

```cpp
- void updateEncoder().
 ```

### Creating a object of Class Encoder

 Meaning of parameters:

- **encodervalue**  - Variable which stores the Encoder value.
- **encoderPin** - Encoder Pin Number connected to Controller.
- **comparePin** - Compare Pin Number connected to Controller.

 ```cpp
Encoder X(int EncoderPin, int ComparePin);
```

### Invoking Member Functions of Class Encoder

```cpp
Encoder X(int EncoderPin, int ComparePin);

X.updateEncoder();     //Updating Encoder Values

```

## 3.Class Mpu

Class Mpu is used to represent the inertial measurement unit MPU6050.

### Member Variables of Class Mpu

```cpp
```

### Member Functions of Class Mpu

```cpp
- int readMpu(int s).
```

### Creating an object of Class Mpu

Meaning of parameters:

- s - to select the serial port.

| s  | serial port |
|--- |-------------|
| 1  | serial1     |
| 2  | serial2     |
| 3  | serial3     |

```cpp
Mpu V;
```

### Invoking Member Functions of Class Mpu

```cpp
Mpu V;

V.readMpu(2);      //reads available data from serial port 2
```
