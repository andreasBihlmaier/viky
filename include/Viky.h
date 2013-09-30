#ifndef _VIKY_H_
#define _VIKY_H_

// system includes

// library includes

// custom includes
#include "MotionControl.h"


// forward declarations



class Viky
{
  public:
    // enums

    // typedefs

    // const static member variables
    const static unsigned motorCount = 3;
    const static unsigned rotationMotorIdx = 0;
    const static unsigned tiltMotorIdx = 1;
    const static unsigned linearMotorIdx = 2;
 
    // static utility functions


    // constructors
    Viky();

    // overwritten methods

    // methods
    bool init();
    void rotate(double pos); // radians [-PI, PI] (=[-180, 180])
    void tilt(double angle); // radians [0, PI/3] (=[0, 60])
    void linear(double pos); // cm [0, 20]

    // variables


  private:
    // methods
    MotionControl* m_motors[motorCount];

    // variables


};

#endif // _VIKY_H_
