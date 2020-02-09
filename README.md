# OOPsOnRobots
Classes and objects implementation.

## Class Piston
--------------------------------
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
Meaning of patameters:
 - **Extension_Pin**    - Pin which is given high to extend the piston.
 - **Retraction_Pin**   - Pin which is given low to retract the piston.

```cpp
Piston P1(int Extension_Pin, int Retraction_Pin);
```

### Invoking Member Functions
```cpp
Piston P1(int Extension_Pin, int Retraction_Pin);

P1.Extend();      //Extending the Piston
P1.Retract();     //Retracting the Piston

```