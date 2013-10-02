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

  viky = new Viky();
  signal(SIGINT, vikyDisableSighandler);

  if (!viky->init()) {
    printf("Viky init() failed\n");
    return 1;
  }

  if (!viky->homing()) {
    printf("Viky homing() failed\n");
    return 2;
  }

  printf("rotate(M_PI/2)\n");
  viky->rotate(M_PI/2);
  getchar();
  printf("rotate(-M_PI/4)\n");
  viky->rotate(-M_PI/4);
  getchar();
  printf("tilt(M_PI/9)\n");
  viky->tilt(M_PI/9);
  getchar();
  printf("tilt(M_PI/18)\n");
  viky->tilt(M_PI/18);
  getchar();
  printf("linear(6)\n");
  viky->linear(6);
  getchar();
  printf("linear(3)\n");
  viky->linear(3);
  getchar();

  printf("TODO\n");
  sleep(100);

  return 0;
}
