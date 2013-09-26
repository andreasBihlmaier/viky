#include "MotionControl.h"

// system includes
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <err.h>
#include <stdexcept>

// library includes

// custom includes


/*---------------------------------- public: -----------------------------{{{-*/
MotionControl::MotionControl(int p_motorID, const std::string& p_ttyDevicePath)
  :m_motorID(p_motorID),
   m_ttyDevicePath(p_ttyDevicePath),
   m_ttyFD(-1),
   m_initialized(false)
{
}

bool
MotionControl::init()
{
  m_ttyFD = open(m_ttyDevicePath.c_str(), O_RDWR);
  if (m_ttyFD == -1) {
    warn("motor%d: Failed to open() tty=%s", m_motorID, m_ttyDevicePath.c_str());
    return false;
  }

  struct termios ttyOptions;
  tcgetattr(m_ttyFD, &ttyOptions);
  cfsetispeed(&ttyOptions, baudrate);
  cfsetospeed(&ttyOptions, baudrate);
  ttyOptions.c_cflag &= ~(PARENB | CSTOPB); // no parity; 1 stop bit
  ttyOptions.c_iflag &= ~(IXON | IXOFF | IXANY); // no HW flow control
  ttyOptions.c_lflag |= ICANON;
  if (tcsetattr(m_ttyFD, TCSANOW, &ttyOptions) != 0) {
    warn("motor%d: Failed to tcsetattr()", m_motorID);
    return false;
  }

  resetMotor();
  enableMotor();
  enableAcks();

  m_initialized = true;
  return true;
}

bool
MotionControl::initialized()
{
  return m_initialized;
}

void
MotionControl::sendCmdAck(const std::string& p_cmd)
{
  if (!initialized()) {
    printf("motor%d: sendCmd(): not initialized\n", m_motorID);
    return;
  }

  sendCmd(p_cmd);
  waitForOKReply();
}

std::string
MotionControl::waitForReply()
{
  if (!initialized()) {
    printf("motor%d: getReply(): not initialized\n", m_motorID);
    return std::string();
  }

  std::string reply = getReply();
  while (reply.empty()) {
    usleep(100);
    reply = getReply();
  }

  printf("waitForReply: %s\n", reply.c_str());

  return reply;
}
/*------------------------------------------------------------------------}}}-*/

/*---------------------------------- private: ----------------------------{{{-*/
void
MotionControl::sendCmd(const std::string& p_cmd)
{
  std::string cmd = p_cmd;
  if (cmd[cmd.size() - 1] != '\r') {
    cmd += '\r';
  }

  //printf("sendCmd (len=%zd): \"%s\"\n", cmd.size(), cmd.c_str());

  int bytesWritten;
  if ((bytesWritten = write(m_ttyFD, cmd.c_str(), cmd.size())) != cmd.size()) {
    throw std::runtime_error("Short write to motor");
  }
}

void
MotionControl::resetMotor()
{
  sendCmd("RESET");
}

void
MotionControl::enableMotor()
{
  sendCmd("EN");
}

void
MotionControl::enableAcks()
{
  sendCmd("ANSW2");
}

std::string
MotionControl::getReply()
{
  char replyChars[maxReplySize];
  int replyLength = read(m_ttyFD, replyChars, maxReplySize);
  if (replyLength < 2 || /*replyChars[replyLength - 2] != '\r' ||*/ replyChars[replyLength - 1] != '\n') {
    replyLength = 0;
  } else {
    replyLength = replyLength - 1; // strip '\n'
  }

  return std::string(replyChars, replyLength);
}

bool
MotionControl::waitForOKReply()
{
  std::string reply = waitForReply();
  if (reply == "OK") {
    return true;
  } else if (reply == "Unknown command"
             || reply == "Invalid parameter"
             || reply == "Command not available"
             || reply == "Overtemperature - drive disabled") {
    printf("got NACK: %s\n", reply.c_str());
    return false;
  } else {
    throw std::runtime_error(std::string("Unknown reply string: ") + reply);
  }
}
/*------------------------------------------------------------------------}}}-*/
