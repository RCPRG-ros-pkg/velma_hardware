
#include <net/if.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>

#include <inttypes.h>

#include <iostream>
#include <cstring>

#include "CANDev.h"

CANDev::CANDev(std::string dev_name) {
  struct sockaddr_can addr;
  struct ifreq ifr;
  
  if((dev = rt_dev_socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
    std::cout<< "Error while opening socket" << std::endl;
    dev = -1;
  }
  
  strcpy(ifr.ifr_name, dev_name.c_str());
  rt_dev_ioctl(dev, SIOCGIFINDEX, &ifr);
  
  addr.can_family  = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex; 
  
  if(rt_dev_bind(dev, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
    std::cout << "Error in socket bind" << std::endl;
    rt_dev_close(dev);
    dev = -1;
  }
}

CANDev::~CANDev() {
  if(dev > -1) {
    rt_dev_close(dev);
  }
}

void CANDev::send(const uint32_t can_id, const uint8_t len, const uint8_t *data){
  struct can_frame frame;
  
  frame.can_id = can_id;
  frame.can_dlc = len;
  
  memcpy(frame.data, data, len);
  rt_dev_send(dev, (void *)&frame, sizeof(frame), 0);
}

uint32_t CANDev::waitForReply(uint32_t can_id, uint8_t *data) {
  struct can_frame frame;
  
  // search frame buffer
  for(size_t i = 0; i < frame_buf.size(); i++) {

    if(frame_buf[i].can_id == can_id) {
      memcpy(data, frame_buf[i].data, frame_buf[i].can_dlc);
      frame_buf.erase(frame_buf.begin()+i);
      return can_id;
    }
  }
  
  // wait for new data
  while(1) {
    size_t ret = rt_dev_recv(dev, (void *)&frame, sizeof(frame), 0);
    if(ret != sizeof(frame)) {
      continue;
    }
    

    if(frame.can_id == can_id) {
     memcpy(data, frame.data, frame.can_dlc);
      return can_id;
    }
    
    frame_buf.push_back(frame);
  }
}

