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
#include <sstream>
#include <iomanip>

#include <sys/ioctl.h>

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
  ttyOptions.c_iflag &= ~(IXON | IXOFF | IXANY); // no HW flow control
  ttyOptions.c_iflag &= ~(IGNCR | INLCR | ICRNL); // input: do not modify '\n' and '\r'
  ttyOptions.c_oflag &= ~(ONLCR | OCRNL | ONLRET | ONOCR); // output: do not modify '\n' and '\r'
  ttyOptions.c_cflag &= ~(PARENB | CSTOPB); // no parity; 1 stop bit
  ttyOptions.c_lflag |= ICANON;
  if (tcsetattr(m_ttyFD, TCSANOW, &ttyOptions) != 0) {
    warn("motor%d: Failed to tcsetattr()", m_motorID);
    return false;
  }

  sendCmd("ANSW0");
  m_initialized = true;

  sendCmd("GTYP");
  printf("Controller: %s\n", getReply().c_str());

  return true;
}

bool
MotionControl::initialized()
{
  return m_initialized;
}

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

std::string
MotionControl::getReply()
{
  failOnUnitialized("getReply()");

  char replyChars[maxReplySize];
  int replyLength = read(m_ttyFD, replyChars, maxReplySize);
  replyChars[replyLength] = '\0';
  //printf("getReply() raw: %s=\"%s\"\n", toHexString(replyChars).c_str(), replyChars);
  if (replyLength < 3 || replyChars[replyLength - 2] != '\r' || replyChars[replyLength - 1] != '\n') {
    replyLength = 0;
  } else {
    replyLength = replyLength - 2; // strip "\r\n"
  }

  return std::string(replyChars, replyLength);
}

std::string
MotionControl::getReplyNoBlock()
{
  failOnUnitialized("getReplyNoBlock()");

  int replyBytesAvailable;
  ioctl(m_ttyFD, FIONREAD, &replyBytesAvailable);
  if (replyBytesAvailable == 0) {
    return std::string();
  }
  return getReply();
}

std::string
MotionControl::getReplyWait(int p_waitMaxMS)
{
  failOnUnitialized("getReplyWait()");

  int waitDuration = 1;
  int waitMaxPeriods = p_waitMaxMS / waitDuration;
  for (int waitPeriods = 0; waitPeriods < waitMaxPeriods; waitPeriods++) {
    std::string reply = getReplyNoBlock();
    if (!reply.empty()) {
      return reply;
    }
    usleep(waitDuration * 1000);
  }

  return std::string();
}
/*------------------------------------------------------------------------}}}-*/

/*---------------------------------- private: ----------------------------{{{-*/
void
MotionControl::failOnUnitialized(const std::string& p_errorMsg)
{
  if (!initialized()) {
    throw std::runtime_error("motor" + toString(m_motorID) + " not initialized: " + p_errorMsg);
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

template<class T> std::string
MotionControl::toString(T p_arg)
{
  std::stringstream ss;

  ss << p_arg;

  return ss.str();
}

std::string
MotionControl::toHexString(const std::string& p_str)
{
  std::stringstream ss;
  ss << "0x";
  for (size_t i = 0; i < p_str.size(); i++) {
    ss << std::setw(2) << std::setfill('0') << std::hex << (unsigned int)p_str[i] << " ";
  }
  return ss.str();
}
/*------------------------------------------------------------------------}}}-*/
