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
    motor.sendCmd(userCmd);
    std::string motorReply = motor.getReplyWait(100);
    if (!motorReply.empty()) {
      std::cout << "Reply (len=" << motorReply.size() << "): " << motorReply << std::endl;
    }
  }

  return 0;
}
