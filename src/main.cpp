#include <string>
#include <iostream>

#include "MotionControl.h"

int
main(int argc, char** argv)
{
  //MotionControl motor(0, "/dev/ttyS3"); // rotation
  //MotionControl motor(1, "/dev/ttyS4"); // neigen
  MotionControl motor(2, "/dev/ttyS5"); // rein/raus
  if (!motor.init()) {
    return 1;
  }

  while (true) {
    std::string userCmd;
    std::cout << "Next command: ";
    std::cin >> userCmd;
    if (userCmd == "CST") {
      std::cout << "Configuration Status: " << motor.getConfigurationStatus() << std::endl;
    } else if (userCmd == "OST") {
      std::cout << "Operation Status: " << motor.getOperationStatus() << std::endl;
    } else {
      motor.sendCmd(userCmd);
      std::string motorReply = motor.getReplyWait(100);
      if (!motorReply.empty()) {
        std::cout << "Reply (len=" << motorReply.size() << "): " << motorReply << std::endl;
      }
    }
  }

  return 0;
}
