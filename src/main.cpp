#include <string>
#include <iostream>

#include "MotionControl.h"

int
main(int argc, char** argv)
{
  MotionControl motor(0, "/dev/ttyS3");
  if (!motor.init()) {
    return 1;
  }

  while (true) {
    std::string userCmd;
    std::cout << "Next command: ";
    std::cin >> userCmd;
    motor.sendCmdAck(userCmd);
    std::string motorReply = motor.waitForReply();
    std::cout << "Reply (len=" << motorReply.size() << "): " << motorReply << std::endl;
  }

  return 0;
}
