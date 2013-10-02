#include <string>
#include <iostream>
#include <signal.h>

#include "MotionControl.h"

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
  //motor = MotionControl(0, "/dev/ttyS3"); // rotation
  //motor = MotionControl(1, "/dev/ttyS4"); // tilt
  motor = new MotionControl(2, "/dev/ttyS5"); // linear
  if (!motor->init()) {
    return 1;
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
