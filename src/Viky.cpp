#include "Viky.h"

// system includes
#include <stdio.h>

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

bool
Viky::homing()
{
  m_motors[tiltMotorIdx]->disableMotor();
  m_motors[linearMotorIdx]->disableMotor();
  printf("Move linear all the way out (0 pos) and tilt to vertical (0 angle). Then press [RETURN] to start homing\n");
  getchar();
  m_motors[tiltMotorIdx]->enableMotor();
  m_motors[linearMotorIdx]->enableMotor();

  /* TODO
  if (!m_motors[tiltMotorIdx]->homing()) {
    printf("Failed to home tilt\n");
    return false;
  }
  */

  if (!m_motors[linearMotorIdx]->homing(linearOutSign)) {
    printf("Failed to home linear\n");
    return false;
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
char
getYesNo()
{
  char answer;
  do {
    printf("y/n? ");
    answer = getchar();
  } while (answer != 'y' && answer != 'n');

  return answer;
}
/*------------------------------------------------------------------------}}}-*/
