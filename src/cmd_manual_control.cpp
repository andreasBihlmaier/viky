#include <string>
#include <iostream>
#include <stdio.h>
#include <signal.h>

#include <ros/ros.h>

#include "MotionControl.h"
#include "Viky.h"

MotionControl* motor;

void
motorDisableSighandler(int)
{
  motor->disableMotor();
  _exit(0);
}

int
main(int argc, char** argv)
{
  if (argc != 2) {
    printf("usage: %s rotation|tilt|linear\n", argv[0]);
    return 1;
  }

  ros::init(argc, argv, "cmd_manual_control");

  std::string motorString(argv[1]);
  if (motorString == "rotation") {
    motor = new MotionControl(Viky::rotationMotorIdx, Viky::rotationMotorDevicePath);
  } else if (motorString == "tilt") {
    motor = new MotionControl(Viky::tiltMotorIdx, Viky::tiltMotorDevicePath);
  } else if (motorString == "linear") {
    motor = new MotionControl(Viky::linearMotorIdx, Viky::linearMotorDevicePath);
  } else {
    printf("Motor unknown: %s\n", motorString.c_str());
    return 2;
  }

  if (!motor->init()) {
    return 3;
  }

  signal(SIGINT, motorDisableSighandler);

  while (true) {
    std::string userCmd;
    std::cout << "Next command: ";
    std::cin >> userCmd;
    if (userCmd == "CST") {
      std::cout << "Configuration Status: " << motor->getConfigurationStatus() << std::endl;
    } else if (userCmd == "OST") {
      std::cout << "Operation Status: " << motor->getOperationStatus() << std::endl;
    } else if (userCmd == "HOMING") {
      motor->homing(1);
    } else {
      motor->sendCmd(userCmd);
      std::string motorReply = motor->getReplyWait(100);
      if (!motorReply.empty()) {
        std::cout << "Reply (len=" << motorReply.size() << "): " << motorReply << std::endl;
      }
    }
  }


  return 0;
}
