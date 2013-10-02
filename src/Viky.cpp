#include "Viky.h"

// system includes
#include <stdio.h>
#include <cmath>

// library includes
#include <ros/ros.h>

// custom includes


/*---------------------------------- public: -----------------------------{{{-*/
const std::string Viky::rotationMotorDevicePath = "/dev/ttyS3";
const std::string Viky::tiltMotorDevicePath = "/dev/ttyS4";
const std::string Viky::linearMotorDevicePath = "/dev/ttyS5";

Viky::Viky()
{
  m_motors[rotationMotorIdx] = new MotionControl(rotationMotorIdx, rotationMotorDevicePath);
  m_motors[tiltMotorIdx] = new MotionControl(tiltMotorIdx, tiltMotorDevicePath);
  m_motors[linearMotorIdx] = new MotionControl(linearMotorIdx, linearMotorDevicePath);
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
Viky::disable()
{
  for (unsigned motorIdx = 0; motorIdx < motorCount; motorIdx++) {
    m_motors[motorIdx]->disableMotor();
  }
}

void
Viky::enable()
{
  for (unsigned motorIdx = 0; motorIdx < motorCount; motorIdx++) {
    m_motors[motorIdx]->enableMotor();
  }
}

bool
Viky::homing()
{
  disable();
  printf("\nHOMING move by hand:\n");
  printf("Rotation to marking at 'EndoControl' label\n");
  printf("Tilt to vertical (0 angle)\n");
  printf("Linear all the way out (0 pos)\n");
  printf("Then press [RETURN] to start homing\n");
  getchar();
  enable();

  m_motors[rotationMotorIdx]->setHomePosition();

  if (!m_motors[tiltMotorIdx]->homing(tiltTowardsVerticalSign)) {
    printf("Failed to home tilt\n");
    return false;
  }

  if (!m_motors[linearMotorIdx]->homing(linearOutSign)) {
    printf("Failed to home linear\n");
    return false;
  }

  return true;
}

void
Viky::rotate(double pos)
{
  if (pos > M_PI || pos < -M_PI) {
    ROS_ERROR("Viky::rotate invalid pos: %lf\n", pos);
    return;
  }
  m_motors[rotationMotorIdx]->movePos(double(rotationFullRotationIncrements)/2 * rotationRightSign * (pos / M_PI));
}

void
Viky::tilt(double angle)
{
  if (angle > M_PI/3 || angle < 0) {
    ROS_ERROR("Viky::tilt invalid angle: %lf\n", angle);
    return;
  }
  m_motors[tiltMotorIdx]->movePos(double(tiltTotalIncrements) * tiltTowardsHorizontalSign * ((angle * 3)/M_PI));
}

void
Viky::linear(double pos)
{
  if (pos > 20 || pos < 0) {
    ROS_ERROR("Viky::linear invalid pos: %lf\n", pos);
    return;
  }
  m_motors[linearMotorIdx]->movePos(double(linearTotalIncrements) * linearInSign * (pos/20.0));
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
