#ifndef _MOTOR_CONTROLLER_H_
#define _MOTOR_CONTROLLER_H_

#include <inttypes.h>
#include "CANDev.h"

#define MODE_ERROR                        0x00
#define MODE_MANUAL                0x01
#define MODE_SPEED                        0x02
#define MODE_CURRENT                0x03
#define MODE_POSITION                0x04
#define MODE_PWM                        0x05
#define MODE_PVT                        0x06
#define MODE_SYNC_PWM0                0x10
#define MODE_SYNC_CURRENT0 0x11
#define MODE_SYNC_POS0                0x12

#define STATUS_LIMITSWITCHUP                (1 << 0)
#define STATUS_LIMITSWITCHDOWN                (1 << 1)
#define STATUS_SYNCHROSWITCH                (1 << 2)
#define STATUS_ENCODERINDEXSIGNAL        (1 << 3)
#define STATUS_SYNCHRONIZED                (1 << 4)
#define STATUS_PVTTRAJACCOMPLISHED        (1 << 5)
#define STATUS_POSITIONLIMIT                (1 << 10)
#define STATUS_SPEEDLIMIT                        (1 << 11)
#define STATUS_CURRENTLIMIT                (1 << 12)
#define STATUS_OVERCURRENT                        (1 << 13)
#define STATUS_POWERSTAGEFAULT                (1 << 14)
#define STATUS_ERROR                                (1 << 15)

#define ISSYNCHRONIZED(x) ((x&STATUS_SYNCHRONIZED)==STATUS_SYNCHRONIZED)

class MotorController {
public:
  MotorController(std::string dev_name = "can0");
  ~MotorController();
  void setMode(const int dev_id,const uint8_t mode);
  
  void setVelocity(const int dev_id, int32_t vel);
  void setVelocity2(const int group_id, int32_t vel1, int32_t vel2);
  
  void setPosition(const int dev_id, int32_t pos);
  void setPosition2(const int group_id, int32_t pos1, int32_t pos2);
  
  void setCurrent(const int dev_id, int16_t cur);

  void getStatus(const int dev_id, uint32_t &status, uint8_t &mode);
  void getStatus2(const int group_id, 
                  uint32_t &status1, uint8_t &mode1,
                  uint32_t &status2, uint8_t &mode2);
  void getStatus4(const int group_id, 
                  uint32_t &status1, uint8_t &mode1,
                  uint32_t &status2, uint8_t &mode2,
                  uint32_t &status3, uint8_t &mode3,
                  uint32_t &status4, uint8_t &mode4);
  
  void getPosition(const int dev_id, int32_t &pos);
  void getPosition2(const int group_id, int32_t &pos1, int32_t &pos2);
  void getPosition4(const int group_id,
                    int32_t &pos1, int32_t &pos2,
                    int32_t &pos3, int32_t &pos4);
  
protected:
private:
  CANDev dev;
};

#endif

