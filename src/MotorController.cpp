
#include "MotorController.h"

#include <cstring>

#define MSG_POS_MSR 48
#define MSG_VEL_MSR 49
#define MSG_POS_AND_VEL_MSR 50

#define MSG_SET_PARAM 53
#define MSG_GET_PARAM 54
#define MSG_GET_PARAM_REP 55

#define MSG_MODE_CMD 56
#define MSG_STATUS 57
#define MSG_SW_VER 58

#define MSG_PWM_CMD 16
#define MSG_TRQ_CMD 17

#define MSG_VEL_CMD 32
#define MSG_POS_CMD 33

#define ADDR_SINGLE 0
#define ADDR_DOUBLE 16
#define ADDR_QUAD   24

  MotorController::MotorController(std::string dev_name) : dev(dev_name) {
  
  }
  
  MotorController::~MotorController() {
  }
  
  void MotorController::setMode(const int dev_id, const uint8_t mode) {
    struct can_frame frame;
    memset(&frame, 0, sizeof(frame));
    
    frame.can_id = (MSG_MODE_CMD << 5) | (dev_id);
    frame.can_dlc = 1;
    
    frame.data[0] = mode;
    
    dev.send(frame.can_id, frame.can_dlc, frame.data);
  }
  
  void MotorController::setCurrent(const int dev_id, int16_t cur) {
	    struct can_frame frame;
	    memset(&frame, 0, sizeof(frame));

	    frame.can_id = (MSG_TRQ_CMD << 5) | dev_id;
	    frame.can_dlc = 8;

	    *(int16_t *)&(frame.data[dev_id%4 * 2]) = cur;

	    dev.send(frame.can_id, frame.can_dlc, frame.data);
  }

  void MotorController::setVelocity(const int dev_id, int32_t vel) {
    struct can_frame frame;
    memset(&frame, 0, sizeof(frame));
    
    frame.can_id = (MSG_VEL_CMD << 5) | dev_id;
    frame.can_dlc = 8;
  
    *(int32_t *)&frame.data[(dev_id%2)*4] = vel;
    
    dev.send(frame.can_id, frame.can_dlc, frame.data);
  }
  
  void MotorController::setVelocity2(const int group_id, int32_t vel1, int32_t vel2) {
    struct can_frame frame;
    memset(&frame, 0, sizeof(frame));
    
    frame.can_id = (MSG_VEL_CMD << 5) | (group_id & 7)| ADDR_DOUBLE;
    frame.can_dlc = 8;
  
    *(int32_t *)&frame.data[0] = vel1;
    *(int32_t *)&frame.data[4] = vel2;
    
    dev.send(frame.can_id, frame.can_dlc, frame.data);
  }
  
  void MotorController::setPosition(const int dev_id, int32_t pos) {
    struct can_frame frame;
    memset(&frame, 0, sizeof(frame));
    
    frame.can_id = (MSG_POS_CMD << 5) | dev_id;
    frame.can_dlc = 8;
  
    *(int32_t *)&frame.data[(dev_id%2)*4] = pos;
    
    dev.send(frame.can_id, frame.can_dlc, frame.data);
  }
  
  void MotorController::setPosition2(const int group_id, int32_t pos1, int32_t pos2) {
    struct can_frame frame;
    memset(&frame, 0, sizeof(frame));
    
    frame.can_id = (MSG_POS_CMD << 5) | (group_id & 7)| ADDR_DOUBLE;
    frame.can_dlc = 8;
  
    *(int32_t *)&frame.data[0] = pos1;
    *(int32_t *)&frame.data[4] = pos2;
    
    dev.send(frame.can_id, frame.can_dlc, frame.data);
  }
  
  void MotorController::getStatus(const int dev_id, uint32_t &status, uint8_t &mode) {
    struct can_frame frame;
    memset(&frame, 0, sizeof(frame));
    
    frame.can_id = (MSG_STATUS << 5) | (dev_id) | CAN_RTR_FLAG;
    frame.can_dlc = 0;
    
    dev.send(frame.can_id, frame.can_dlc, frame.data);
    
    unsigned int can_id;
    can_id = ((MSG_STATUS << 5) | (dev_id));
    dev.waitForReply(can_id, frame.data);
    
    status = *(int32_t*)&frame.data[0];
    mode = frame.data[4];
  }
  
  void MotorController::getStatus2(const int group_id,
                                    uint32_t &status1, uint8_t &mode1,
                                    uint32_t &status2, uint8_t &mode2) {
    struct can_frame frame;
    memset(&frame, 0, sizeof(frame));
    
    frame.can_id = (MSG_STATUS << 5) | (group_id) | CAN_RTR_FLAG | ADDR_DOUBLE;
    frame.can_dlc = 0;
    
    dev.send(frame.can_id, frame.can_dlc, frame.data);
    
    unsigned int can_id;
    can_id = ((MSG_STATUS << 5) | (group_id));
    dev.waitForReply(can_id, frame.data);
    
    status1 = *(int32_t*)&frame.data[0];
    mode1 = frame.data[4];
    
    can_id = ((MSG_STATUS << 5) | (group_id+1));
    dev.waitForReply(can_id, frame.data);
    
    status2 = *(int32_t*)&frame.data[0];
    mode2 = frame.data[4];
  }
  
  void MotorController::getStatus4(const int group_id,
                                    uint32_t &status1, uint8_t &mode1,
                                    uint32_t &status2, uint8_t &mode2,
                                    uint32_t &status3, uint8_t &mode3,
                                    uint32_t &status4, uint8_t &mode4) {
    struct can_frame frame;
    memset(&frame, 0, sizeof(frame));
    
    frame.can_id = (MSG_STATUS << 5) | (group_id) | CAN_RTR_FLAG | ADDR_QUAD;
    frame.can_dlc = 0;
    
    dev.send(frame.can_id, frame.can_dlc, frame.data);
    
    unsigned int can_id;
    can_id = ((MSG_STATUS << 5) | (group_id));
    dev.waitForReply(can_id, frame.data);
    
    status1 = *(int32_t*)&frame.data[0];
    mode1 = frame.data[4];
    
    can_id = ((MSG_STATUS << 5) | (group_id+1));
    dev.waitForReply(can_id, frame.data);
    
    status2 = *(int32_t*)&frame.data[0];
    mode2 = frame.data[4];
    
    can_id = ((MSG_STATUS << 5) | (group_id+2));
    dev.waitForReply(can_id, frame.data);
    
    status3 = *(int32_t*)&frame.data[0];
    mode3 = frame.data[4];
    
    can_id = ((MSG_STATUS << 5) | (group_id+3));
    dev.waitForReply(can_id, frame.data);
    
    status4 = *(int32_t*)&frame.data[0];
    mode4 = frame.data[4];
  }
  
  void MotorController::getPosition(const int dev_id, int32_t &pos) {
    struct can_frame frame;
    memset(&frame, 0, sizeof(frame));
    
    frame.can_id = (MSG_POS_MSR << 5) | (dev_id) | CAN_RTR_FLAG;
    frame.can_dlc = 0;
    
    dev.send(frame.can_id, frame.can_dlc, frame.data);
    
    unsigned int can_id;
    can_id = ((MSG_POS_MSR << 5) | (dev_id));
    
    dev.waitForReply(can_id, frame.data);
    
    pos = *(int32_t*)&frame.data[0];
  }
  
  void MotorController::getPosition2(const int group_id, int32_t &pos1, int32_t &pos2) {
    struct can_frame frame;
    memset(&frame, 0, sizeof(frame));
    
    frame.can_id = (MSG_POS_MSR << 5) | (group_id) | CAN_RTR_FLAG | ADDR_DOUBLE;
    frame.can_dlc = 0;
    
    dev.send(frame.can_id, frame.can_dlc, frame.data);
    
    unsigned int can_id;
    can_id = ((MSG_POS_MSR << 5) | (group_id));
    
    dev.waitForReply(can_id, frame.data);
    
    pos1 = *(int32_t*)&frame.data[0];
    
    can_id = ((MSG_POS_MSR << 5) | (group_id+1));
    
    dev.waitForReply(can_id, frame.data);
    
    pos2 = *(int32_t*)&frame.data[0];
  }
  
  void MotorController::getPosition4(const int group_id, int32_t &pos1, int32_t &pos2, int32_t &pos3, int32_t &pos4) {
    struct can_frame frame;
    memset(&frame, 0, sizeof(frame));
    
    frame.can_id = (MSG_POS_MSR << 5) | (group_id) | CAN_RTR_FLAG | ADDR_QUAD;
    frame.can_dlc = 0;
    
    dev.send(frame.can_id, frame.can_dlc, frame.data);
    
    unsigned int can_id;
    can_id = ((MSG_POS_MSR << 5) | (group_id));
    
    dev.waitForReply(can_id, frame.data);
    
    pos1 = *(int32_t*)&frame.data[0];
    
    can_id = ((MSG_POS_MSR << 5) | (group_id+1));
    
    dev.waitForReply(can_id, frame.data);
    
    pos2 = *(int32_t*)&frame.data[0];
    
    can_id = ((MSG_POS_MSR << 5) | (group_id+2));
    
    dev.waitForReply(can_id, frame.data);
    
    pos3 = *(int32_t*)&frame.data[0];
    
    can_id = ((MSG_POS_MSR << 5) | (group_id+3));
    
    dev.waitForReply(can_id, frame.data);
    
    pos4 = *(int32_t*)&frame.data[0];
  }
  
