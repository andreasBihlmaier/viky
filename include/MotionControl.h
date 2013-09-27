#ifndef _MOTION_CONTROL_H_
#define _MOTION_CONTROL_H_

// system includes
#include <string>

// library includes

// custom includes


// forward declarations



class MotionControl
{
  public:
    // enums

    // typedefs

    // const static member variables
    const static unsigned long baudrate = 9600UL;
    const static int maxReplySize = 64;
 
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

    // variables


  private:
    // methods
    void resetMotor();
    void enableMotor();
    template<class T> std::string toString(T p_arg);
    std::string toHexString(const std::string& p_str);
    void failOnUnitialized(const std::string& p_errorMsg);

    // variables
    int m_motorID;
    std::string m_ttyDevicePath;
    int m_ttyFD;
    bool m_initialized;


};

#endif // _MOTION_CONTROL_H_
