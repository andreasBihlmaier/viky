#include "Viky.h"

// system includes
#include <stdio.h>
#include <cmath>

// library includes

// custom includes


/*---------------------------------- public: -----------------------------{{{-*/
const std::string Viky::rotationMotorDevicePath = "/dev/ttyS3";
const std::string Viky::tiltMotorDevicePath = "/dev/ttyS4";
const std::string Viky::linearMotorDevicePath = "/dev/ttyS5";
const double Viky::rOffset = 0.03;
const double Viky::thetaStart = M_PI;

Viky::Viky()
{
  m_motors[rotationMotorIdx] = new MotionControl(rotationMotorIdx, rotationMotorDevicePath);
  m_motors[tiltMotorIdx] = new MotionControl(tiltMotorIdx, tiltMotorDevicePath);
  m_motors[linearMotorIdx] = new MotionControl(linearMotorIdx, linearMotorDevicePath);
}

bool
Viky::initHardware()
{
  for (unsigned motorIdx = 0; motorIdx < motorCount; motorIdx++) {
    if (!m_motors[motorIdx]->init()) {
      return false;
    }
  }

  return true;
}

bool
Viky::initROS()
{
  m_jointsSubscriber = m_nodeHandle.subscribe<sensor_msgs::JointState>("set_joint", 1, &Viky::jointsCallback, this);
  m_jointsPublisher = m_nodeHandle.advertise<sensor_msgs::JointState>("get_joint", 1);
  m_trocarSubscriber = m_nodeHandle.subscribe<trocar2cartesian_msgs::TrocarPose>("set_trocar", 1, &Viky::trocarCallback, this);
  m_trocarPublisher = m_nodeHandle.advertise<trocar2cartesian_msgs::TrocarPose>("get_trocar", 1);

  m_publishThread = new boost::thread(boost::bind(&Viky::publishJoints, this));

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
  printf("Rotation of 'EndoControl' label to negative world X-axis\n");
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

double
Viky::getRotation()
{
  int64_t increments = m_motors[rotationMotorIdx]->getPos();
  return (double(increments) * rotationRightSign * M_PI) / (rotationFullRotationIncrements/2.0);
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

double
Viky::getTilt()
{
  int64_t increments = m_motors[tiltMotorIdx]->getPos();
  return (double(increments) * tiltTowardsHorizontalSign * M_PI) / (tiltTotalIncrements * 3.0);
}

void
Viky::linear(double pos)
{
  if (pos > 0.2 || pos < 0) {
    ROS_ERROR("Viky::linear invalid pos: %lf\n", pos);
    return;
  }
  m_motors[linearMotorIdx]->movePos(double(linearTotalIncrements) * linearInSign * (pos/0.2));
}

double
Viky::getLinear()
{
  int64_t increments = m_motors[linearMotorIdx]->getPos();
  return (double(increments) * linearInSign * 0.2) / double(linearTotalIncrements);
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

void
Viky::jointsCallback(const sensor_msgs::JointStateConstPtr& msg)
{
  if (msg->position.size() != motorCount) {
    ROS_ERROR("Wrong number of joint positions: %zd\n", msg->position.size());
    return;
  }

  rotate(msg->position[rotationMotorIdx]);
  tilt(msg->position[tiltMotorIdx]);
  linear(msg->position[linearMotorIdx]);
}

void
Viky::trocarCallback(const trocar2cartesian_msgs::TrocarPoseConstPtr& msg)
{
  linear(msg->r - rOffset);
  tilt(thetaStart - msg->theta);
  rotate(-msg->phi);
}

void
Viky::publishJoints()
{
  uint32_t sequence = 0;
  ros::Rate publish_rate(100);
  ROS_INFO_STREAM("ViKY starting to publish");
  while (ros::ok()) {
    sensor_msgs::JointState joints;
    joints.header.seq = sequence;
    joints.header.stamp = ros::Time::now();
    joints.name.push_back("rotation");
    joints.position.push_back(getRotation());
    joints.name.push_back("tilt");
    joints.position.push_back(getTilt());
    joints.name.push_back("linear");
    joints.position.push_back(getLinear());
    //ROS_INFO_STREAM(joints);
    m_jointsPublisher.publish(joints);

    trocar2cartesian_msgs::TrocarPose trocar;
    trocar.instrument_tip_frame = "endoscope";
    trocar.r = getLinear() + rOffset;
    trocar.theta = thetaStart - getTilt();
    trocar.phi = -getRotation();
    m_trocarPublisher.publish(trocar);

    publish_rate.sleep();
    sequence++;
  }
}
/*------------------------------------------------------------------------}}}-*/
