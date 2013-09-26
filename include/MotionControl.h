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
    void sendCmdAck(const std::string& p_cmd);
    std::string waitForReply();

    // variables


  private:
    // methods
    void sendCmd(const std::string& p_cmd);
    void resetMotor();
    void enableMotor();
    void enableAcks();
    std::string getReply();
    bool waitForOKReply();

    // variables
    int m_motorID;
    std::string m_ttyDevicePath;
    int m_ttyFD;
    bool m_initialized;


};

#endif // _MOTION_CONTROL_H_
