#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>

#include <vector>
#include <Eigen/Dense>

#include "MotorController.h"

#define TORSO_MNJ 2

#define OFFSET0 0
#define OFFSET1 280

#define GEAR0 158.0
#define GEAR1 1.0

#define ENC0 5000.0
#define ENC1 5000.0

class VelmaTorso : public RTT::TaskContext {
public:
  VelmaTorso(const std::string & name) : TaskContext(name), mc("rtcan0") {

    prop_device = "rtcan0";
    synchro_ = false;
    this->addProperty("device", prop_device);

    this->ports()->addPort("JointPositionCommand", port_JointPositionCommand).doc("");
    this->ports()->addPort("JointTorqueCommand", port_JointTorqueCommand).doc("");

    this->ports()->addPort("JointVelocity", port_JointVelocity).doc("");
    this->ports()->addPort("JointPosition", port_JointPosition).doc("");
  }

  ~VelmaTorso(){
  }

  bool configureHook() {

    jnt_pos_.resize(TORSO_MNJ);
    jnt_vel_.resize(TORSO_MNJ);
    jnt_pos_cmd_.resize(TORSO_MNJ);
    jnt_trq_cmd_.resize(TORSO_MNJ);

    port_JointPosition.setDataSample(jnt_pos_);
    port_JointVelocity.setDataSample(jnt_vel_);

    return true;
  }

  bool startHook() {
    synchro_ = true;

    jnt_trq_cmd_(0) = 0.0;

    mc.setMode(0, MODE_CURRENT);
    mc.setMode(1, MODE_POSITION);
    mc.setMode(2, MODE_POSITION);
    mc.setMode(3, MODE_POSITION);
    return true;
  }

  void stopHook() {

  }

  void updateHook() {
    
    int32_t pos0, pos1, pos2, pos3, vel0;
    
    uint32_t status0, status1, status2, status3;
    uint8_t mode0, mode1, mode2, mode3;
    
    mc.getStatus4(0, status0, mode0, status1, mode1, status2, mode2, status3, mode3);
    
    if(ISSYNCHRONIZED(status0) && ISSYNCHRONIZED(status1) && ISSYNCHRONIZED(status2) && ISSYNCHRONIZED(status3)) {
      
      //mc.setPosition2(0, 0, 250);
      //mc.setVelocity2(0, 0,  50);

      port_JointTorqueCommand.read(jnt_trq_cmd_);

      int16_t trq0 = -((jnt_trq_cmd_(0)/GEAR0)/0.105) * 1.2 * 1000;

      if(trq0 < 0.0) {
    	  trq0 -= 500;
      } else {
    	  trq0 += 500;
      }

      mc.setCurrent(0, trq0);

      mc.getPosition4(0, pos0, pos1, pos2, pos3);
      
      vel0 = pos0 - pos_old_;
      pos_old_ = pos0;

      //std::cout << vel0 << std::endl;

      jnt_pos_[0] = -((double)(pos0 - OFFSET0)/(ENC0 * 4.0 * GEAR0) * M_PI * 2);
      jnt_pos_[1] = -((double)(pos1 - OFFSET1)/(ENC1 * 4.0 * GEAR1) * M_PI * 2) - M_PI/2.0;
      
      jnt_vel_[0] = 0;
      jnt_vel_[1] = 0;

      port_JointPosition.write(jnt_pos_);
      port_JointVelocity.write(jnt_vel_);
    } else if(synchro_) {
      if(!ISSYNCHRONIZED(status0)) {
        if(!(mode0 == MODE_SYNC_POS0)) {
          std::cout << "synchronizing axis 0" << std::endl;
          mc.setMode(0, MODE_SYNC_POS0);
        }
      } else if(!ISSYNCHRONIZED(status1)) {
        if(!(mode1 == MODE_SYNC_POS0)) {
          std::cout << "synchronizing axis 1 mode: " << (int)mode1 << std::endl;
          mc.setMode(1, MODE_SYNC_POS0);
        }
      } else if(!ISSYNCHRONIZED(status2)) {
        if(!(mode2 == MODE_SYNC_POS0)) {
          std::cout << "synchronizing axis 2" << std::endl;
          mc.setMode(2, MODE_SYNC_POS0);
        }
      } else if(!ISSYNCHRONIZED(status3)) {
        if(!(mode3 == MODE_SYNC_POS0)) {
          std::cout << "synchronizing axis 3" << std::endl;
          mc.setMode(3, MODE_SYNC_POS0);
        }
      } else {
        mc.setMode(0, MODE_CURRENT);
        mc.setMode(1, MODE_POSITION);
        mc.setMode(2, MODE_POSITION);
        mc.setMode(3, MODE_POSITION);
        synchro_ = false;
      }
    }
  }

private:

  MotorController mc;

  RTT::InputPort<Eigen::VectorXd > port_JointPositionCommand;
  RTT::InputPort<Eigen::VectorXd > port_JointTorqueCommand;

  RTT::OutputPort<Eigen::VectorXd > port_JointVelocity;
  RTT::OutputPort<Eigen::VectorXd > port_JointPosition;

  std::string prop_device;

  Eigen::VectorXd jnt_pos_;
  Eigen::VectorXd jnt_vel_;

  Eigen::VectorXd jnt_pos_cmd_;
  Eigen::VectorXd jnt_trq_cmd_;
  
  bool synchro_;

  int32_t pos_old_;

};

ORO_CREATE_COMPONENT(VelmaTorso)

