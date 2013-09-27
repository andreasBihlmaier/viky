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
const RegisterBits<uint16_t> MotionControl::CST_ANSW_Bits = RegisterBits<uint16_t>(1, 2);
const RegisterBits<uint16_t> MotionControl::CST_SOR_Bits = RegisterBits<uint16_t>(3, 5);
const RegisterBits<uint16_t> MotionControl::CST_MOD_Bits = RegisterBits<uint16_t>(7, 9);
const RegisterBits<uint16_t> MotionControl::CST_EN_Bits = RegisterBits<uint16_t>(10, 10);
const RegisterBits<uint16_t> MotionControl::CST_PR_Bits = RegisterBits<uint16_t>(11, 11);
const RegisterBits<uint16_t> MotionControl::CST_ADIR_Bits = RegisterBits<uint16_t>(12, 12);
const RegisterBits<uint16_t> MotionControl::CST_APL_Bits = RegisterBits<uint16_t>(13, 13);
const RegisterBits<uint16_t> MotionControl::CST_SIN_Bits = RegisterBits<uint16_t>(14, 14);
const RegisterBits<uint16_t> MotionControl::CST_NET_Bits = RegisterBits<uint16_t>(15, 15);

const RegisterBits<uint16_t> MotionControl::OST_Homing_Bits = RegisterBits<uint16_t>(0, 0);
const RegisterBits<uint16_t> MotionControl::OST_ProgRunning_Bits = RegisterBits<uint16_t>(1, 1);
const RegisterBits<uint16_t> MotionControl::OST_ProgStopDelay_Bits = RegisterBits<uint16_t>(2, 2);
const RegisterBits<uint16_t> MotionControl::OST_ProgStopNotify_Bits = RegisterBits<uint16_t>(3, 3);
const RegisterBits<uint16_t> MotionControl::OST_CurrentLimit_Bits = RegisterBits<uint16_t>(4, 4);
const RegisterBits<uint16_t> MotionControl::OST_DeviationError_Bits = RegisterBits<uint16_t>(5, 5);
const RegisterBits<uint16_t> MotionControl::OST_OverVolt_Bits = RegisterBits<uint16_t>(6, 6);
const RegisterBits<uint16_t> MotionControl::OST_OverTemp_Bits = RegisterBits<uint16_t>(7, 7);
const RegisterBits<uint16_t> MotionControl::OST_StatusInput_Bits = RegisterBits<uint16_t>(8, 12);
const RegisterBits<uint16_t> MotionControl::OST_PosReached_Bits = RegisterBits<uint16_t>(16, 16);
const RegisterBits<uint16_t> MotionControl::OST_LimitToContCurrent_Bits = RegisterBits<uint16_t>(17, 17);


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

  sendCmd("CONTMOD");
  sendCmd("ENCRES1000");
  sendCmd("KN1506");
  sendCmd("RM3400");

  sendCmd("SP1500");
  sendCmd("AC1500");
  sendCmd("DEC1500");
  sendCmd("DEV100");
  sendCmd("DCE20");

  sendCmd("EN");

  m_initialized = true;

  sendCmd("GTYP");
  printf("Controller: %s\n", getReply().c_str());

  printf("Status: %s\n", getConfigurationStatus().c_str());

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

std::string
MotionControl::getConfigurationStatus()
{
  sendCmd("CST");
  std::string replyStr = getReply();
  uint16_t reply = toIntSlow<uint16_t>(replyStr);

  std::string cst;
  cst += "ANSW=" + toString(CST_ANSW_Bits.getVal(reply));
  cst += " SOR=" + toString(CST_SOR_Bits.getVal(reply));
  cst += " MOD=" + toString(CST_MOD_Bits.getVal(reply));
  cst += " EN=" + toString(CST_EN_Bits.getVal(reply));
  cst += " PR=" + toString(CST_PR_Bits.getVal(reply));
  cst += " ADIR=" + toString(CST_ADIR_Bits.getVal(reply));
  cst += " APL=" + toString(CST_APL_Bits.getVal(reply));
  cst += " SIN=" + toString(CST_SIN_Bits.getVal(reply));
  cst += " NET=" + toString(CST_NET_Bits.getVal(reply));

  return cst;
}

std::string
MotionControl::getOperationStatus()
{
  sendCmd("OST");
  std::string replyStr = getReply();
  uint32_t reply = toIntSlow<uint32_t>(replyStr);

  std::string ost;
  ost += "Homing=" + toString(OST_Homing_Bits.getVal(reply));
  ost += " ProgRunning=" + toString(OST_ProgRunning_Bits.getVal(reply));
  ost += " ProgStopDelay=" + toString(OST_ProgStopDelay_Bits.getVal(reply));
  ost += " ProgStopNotify=" + toString(OST_ProgStopNotify_Bits.getVal(reply));
  ost += " CurrentLimit=" + toString(OST_CurrentLimit_Bits.getVal(reply));
  ost += " DeviationError=" + toString(OST_DeviationError_Bits.getVal(reply));
  ost += " OverVolt=" + toString(OST_OverVolt_Bits.getVal(reply));
  ost += " OverTemp=" + toString(OST_OverTemp_Bits.getVal(reply));
  ost += " StatusInput=" + toString(OST_StatusInput_Bits.getVal(reply));
  ost += " PosReached=" + toString(OST_PosReached_Bits.getVal(reply));
  ost += " LimitToContCurrent=" + toString(OST_LimitToContCurrent_Bits.getVal(reply));

  return ost;
}

uint64_t
MotionControl::getPos()
{
  sendCmd("POS");
  std::string replyStr = getReply();
  return toIntSlow<uint64_t>(replyStr);
}

void
MotionControl::movePos(uint64_t pos)
{
  sendCmd("LA" + toString(pos));
  sendCmd("M");
}

void
MotionControl::moveStop()
{
  sendCmd("V0");
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

void
MotionControl::disableMotor()
{
  sendCmd("DI");
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


template<class T> T
MotionControl::toIntSlow(const std::string& p_str)
{
  std::stringstream ss;
  ss << p_str;
  T res;
  ss >> res;

  return res;
}
/*------------------------------------------------------------------------}}}-*/
