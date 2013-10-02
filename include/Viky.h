#ifndef _VIKY_H_
#define _VIKY_H_

// system includes

// library includes
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <boost/thread/thread.hpp>

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
    const static std::string rotationMotorDevicePath;
    const static unsigned tiltMotorIdx = 1;
    const static std::string tiltMotorDevicePath;
    const static unsigned linearMotorIdx = 2;
    const static std::string linearMotorDevicePath;

    const static int8_t rotationRightSign = -1;
    const static int8_t rotationLeftSign = 1;
    const static int64_t rotationFullRotationIncrements = 2597500;

    const static int8_t tiltTowardsVerticalSign = 1;
    const static int8_t tiltTowardsHorizontalSign = -1;
    const static int64_t tiltTotalIncrements = 620000;
 
    const static int8_t linearOutSign = 1;
    const static int8_t linearInSign = -1;
    const static int64_t linearTotalIncrements = 1185000;


    // static utility functions


    // constructors
    Viky(const std::string& p_jointSubscribeTopic, const std::string& p_jointPublishTopic);

    // overwritten methods

    // methods
    bool initHardware();
    bool initROS();
    void disable();
    void enable();
    bool homing();
    void rotate(double pos); // radians [-PI, PI] (=[-180, 180]); zero pos = at marking next to "EndoControl" label
    double getRotation();
    void tilt(double angle); // radians [0, PI/3] (=[0, 60]); zero pos = vertical
    double getTilt();
    void linear(double pos); // cm [0, 20]; zero pos = all the way out
    double getLinear();

    // variables


  private:
    // methods
    char getYesNo();
    void jointsCallback(const sensor_msgs::JointStateConstPtr& msg);
    void publishJoints();

    // variables
    MotionControl* m_motors[motorCount];
    std::string m_jointSubscribeTopic;
    std::string m_jointPublishTopic;
    ros::NodeHandle m_nodeHandle;
    ros::Subscriber m_jointsSubscriber;
    ros::Publisher m_jointsPublisher;

    boost::thread* m_publishThread;

};

#endif // _VIKY_H_
