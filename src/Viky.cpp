#include "Viky.h"

// system includes

// library includes

// custom includes


/*---------------------------------- public: -----------------------------{{{-*/
Viky::Viky()
{
  m_motors[rotationMotorIdx] = new MotionControl(0, "/dev/ttyS3");
  m_motors[tiltMotorIdx] = new MotionControl(1, "/dev/ttyS4");
  m_motors[linearMotorIdx] = new MotionControl(2, "/dev/ttyS5");
}

bool
Viky::init()
{
  for (unsigned motorIdx = 0; motorIdx < motorCount; motorIdx++) {
    if (!m_motors[motorIdx]->init()) {
      return false;
    }
  }

  return true;
}

void
Viky::rotate(double pos)
{
  //TODO
}

void
Viky::tilt(double angle)
{
  //TODO
}

void
Viky::linear(double pos)
{
  //TODO
}
/*------------------------------------------------------------------------}}}-*/

/*---------------------------------- private: ----------------------------{{{-*/
/*------------------------------------------------------------------------}}}-*/
