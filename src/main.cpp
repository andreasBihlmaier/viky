#include <string>
#include <iostream>
#include <stdio.h>
#include <signal.h>

#include <ros/ros.h>

#include "Viky.h"

Viky* viky;

void
vikyDisableSighandler(int)
{
  viky->disable();
  _exit(0);
}

int
main(int argc, char** argv)
{
  ros::init(argc, argv, "viky");

  std::string jointsSubscribeTopic = "viky/set_joint";
  std::string jointsPublishTopic = "viky/get_joint";

  viky = new Viky(jointsSubscribeTopic, jointsPublishTopic);
  signal(SIGINT, vikyDisableSighandler);

  if (!viky->initHardware()) {
    printf("Viky initHardware() failed\n");
    return 1;
  }

  if (!viky->homing()) {
    printf("Viky homing() failed\n");
    return 2;
  }

  if (!viky->initROS()) {
    printf("Viky initROS() failed\n");
    return 3;
  }

  ROS_INFO_STREAM("ViKY READY");
  ros::spin();

  return 0;
}
