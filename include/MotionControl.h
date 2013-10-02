#ifndef _MOTION_CONTROL_H_
#define _MOTION_CONTROL_H_

// system includes
#include <string>
#include <stdint.h>

// library includes

// custom includes


// forward declarations

/* to get bits 1-2: startBit=1, endBit=2 */
template<class T>
class RegisterBits {
  public:
    RegisterBits(unsigned p_startBit, unsigned p_endBit)
      :m_startBit(p_startBit),
       m_endBit(p_endBit)
    {
      m_mask = 0;
      for (unsigned bit = m_startBit; bit <= m_endBit; bit++) {
        m_mask |= (1 << bit);
      }
    }

    T
    getVal(T bits) const
    {
      return (bits & m_mask) >> m_startBit;
    } 

    void
    setVal(T& bits, T val) const
    {
      return (bits & ~m_mask) | ((val << m_startBit) & m_mask);
    } 

  private:
    unsigned m_startBit; // inclusive
    unsigned m_endBit; // inclusive
    T m_mask; // 1 between start and end; 0 otherwise
};


class MotionControl
{
  public:
    // enums

    // typedefs

    // const static member variables
    const static unsigned long baudrate = 9600UL;
    const static int maxReplySize = 64;

    const static RegisterBits<uint16_t> CST_ANSW_Bits;
    const static RegisterBits<uint16_t> CST_SOR_Bits;
    const static RegisterBits<uint16_t> CST_MOD_Bits;
    const static RegisterBits<uint16_t> CST_EN_Bits;
    const static RegisterBits<uint16_t> CST_PR_Bits;
    const static RegisterBits<uint16_t> CST_ADIR_Bits;
    const static RegisterBits<uint16_t> CST_APL_Bits;
    const static RegisterBits<uint16_t> CST_SIN_Bits;
    const static RegisterBits<uint16_t> CST_NET_Bits;

    const static RegisterBits<uint16_t> OST_Homing_Bits;
    const static RegisterBits<uint16_t> OST_ProgRunning_Bits;
    const static RegisterBits<uint16_t> OST_ProgStopDelay_Bits;
    const static RegisterBits<uint16_t> OST_ProgStopNotify_Bits;
    const static RegisterBits<uint16_t> OST_CurrentLimit_Bits;
    const static RegisterBits<uint16_t> OST_DeviationError_Bits;
    const static RegisterBits<uint16_t> OST_OverVolt_Bits;
    const static RegisterBits<uint16_t> OST_OverTemp_Bits;
    const static RegisterBits<uint16_t> OST_StatusInput_Bits;
    const static RegisterBits<uint16_t> OST_PosReached_Bits;
    const static RegisterBits<uint16_t> OST_LimitToContCurrent_Bits;

    const static unsigned homingMeasurementMaxCnt = 500;
    const static int homingCurrentThreshold = 700;

 
    // static utility functions


    // constructors
    MotionControl(int p_motorID, const std::string& p_ttyDevicePath);

    // overwritten methods

    // methods
    bool init();
    bool initialized();
    void sendCmd(const std::string& p_cmd);
    std::string getReply();
    std::string getReplyNoBlock();
    std::string getReplyWait(int p_waitMaxMS);
    std::string getConfigurationStatus();
    std::string getOperationStatus();
    void enableMotor();
    void disableMotor();
    uint64_t getPos();
    void movePos(uint64_t pos);
    void moveStop();
    int getCurrent();
    bool homing(int8_t dir);

    // variables


  private:
    // methods
    void resetMotor();
    void setHomePosition();
    void failOnUnitialized(const std::string& p_errorMsg);

    template<class T> std::string toString(T p_arg);
    std::string toHexString(const std::string& p_str);
    template<class T> T toIntSlow(const std::string& p_str);

    // variables
    int m_motorID;
    std::string m_ttyDevicePath;
    int m_ttyFD;
    bool m_initialized;


};

#endif // _MOTION_CONTROL_H_
