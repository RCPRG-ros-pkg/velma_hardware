#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>

#include <vector>
#include <Eigen/Dense>

#include "MotorController.h"

#define TORSO_MNJ 4
#define POS_CTRL_MNJ 3
#define TRQ_CTRL_MNJ 1

#define OFFSET0 0
#define OFFSET1 280
#define OFFSET2 0
#define OFFSET3 0

#define GEAR0 158.0
#define GEAR1 1.0
#define GEAR2 50.0
#define GEAR3 50.0

#define ENC0 5000.0
#define ENC1 5000.0
#define ENC2 500.0
#define ENC3 500.0

//#define MAX_VEL0
#define MAX_VEL1 250
#define MAX_VEL2 50000
#define MAX_VEL3 20000

class VelmaTorso : public RTT::TaskContext {
public:
	VelmaTorso(const std::string & name) : TaskContext(name), mc("rtcan0") {
		prop_device = "rtcan0";
		this->addProperty("device", prop_device);

		this->ports()->addPort("JointPositionCommand", port_JointPositionCommand).doc("");
		this->ports()->addPort("JointTorqueCommand", port_JointTorqueCommand).doc("");

		this->ports()->addPort("JointVelocity", port_JointVelocity).doc("");
		this->ports()->addPort("JointPosition", port_JointPosition).doc("");
	}

	~VelmaTorso() {
	}

	bool configureHook() {
		jnt_pos_.resize(TORSO_MNJ);
		jnt_vel_.resize(TORSO_MNJ);
		jnt_pos_cmd_.resize(POS_CTRL_MNJ);
		jnt_trq_cmd_.resize(TRQ_CTRL_MNJ);

		port_JointPosition.setDataSample(jnt_pos_);
		port_JointVelocity.setDataSample(jnt_vel_);

		return true;
	}

	bool startHook() {
		// Just to be sure
		drivesSynchronized = false;

		jnt_trq_cmd_(0) = 0.0;

		mc.setMode(0, MODE_CURRENT);
		mc.setMode(1, MODE_POSITION);
		mc.setMode(2, MODE_POSITION);
		mc.setMode(3, MODE_POSITION);
		mc.setVelocity(1, MAX_VEL1);
		mc.setVelocity(2, MAX_VEL2);
		mc.setVelocity(3, MAX_VEL3);
		return true;
	}

	void stopHook() {
	}

	void updateHook() {
		int32_t pos0, pos1, pos2, pos3;
		uint32_t status0, status1, status2, status3;
		uint8_t mode0, mode1, mode2, mode3;
		RTT::FlowStatus pos_fs, trq_fs;

		mc.getStatus4(0, status0, mode0, status1, mode1, status2, mode2, status3, mode3);

		if(ISSYNCHRONIZED(status0) && ISSYNCHRONIZED(status1) && ISSYNCHRONIZED(status2) && ISSYNCHRONIZED(status3) && drivesSynchronized) {
		// All axes synchronized. Proceed with torque or position control.
		
			// Read input ports
			pos_fs = port_JointPositionCommand.read(jnt_pos_cmd_);
			trq_fs = port_JointTorqueCommand.read(jnt_trq_cmd_);
			
			
			if(pos_fs == RTT::NewData){
				int32_t jnt_pos[3];
				
				jnt_pos[0] = -((jnt_pos_cmd_(0) + M_PI/2.0)/(M_PI * 2)) * (ENC1 * 4.0 * GEAR1) + OFFSET1;
				jnt_pos[1] = -(jnt_pos_cmd_(1)/(M_PI * 2)) * (ENC2 * 4.0 * GEAR2) + OFFSET2;
				jnt_pos[2] = -(jnt_pos_cmd_(2)/(M_PI * 2)) * (ENC3 * 4.0 * GEAR3) + OFFSET3;
				
				// New position data: make a new head position command
				//std::cout<<"jnt_pos_cmd_ "<<jnt_pos_cmd_(0)<<" "<<jnt_pos_cmd_(1)<<" "<<jnt_pos_cmd_(2)<< std::endl;
				
				// Send data from JointPositionCommand port #1 and #2 to drives #2 and #3 (group #1)
				mc.setPosition2(1, jnt_pos[1], jnt_pos[2]);
				// Send data from JointPositionCommand port #0 to drive #1
				mc.setPosition(1, jnt_pos[0]);
			}

			if(trq_fs == RTT::NewData){
				// New torque data: make a new torso torque and position command
				int16_t trq0 = -((jnt_trq_cmd_(0)/GEAR0)/0.105) * 1.2 * 1000;
				if(trq0 < 0.0) {
					trq0 -= 500;
				} else {
					trq0 += 500;
				}
				// Send data computed from JointTorqueCommand port #0 to drive #0
				mc.setCurrent(0, trq0);
			}

			mc.getPosition4(0, pos0, pos1, pos2, pos3);

			jnt_pos_[0] = -((double)(pos0 - OFFSET0)/(ENC0 * 4.0 * GEAR0) * M_PI * 2);
			jnt_pos_[1] = -((double)(pos1 - OFFSET1)/(ENC1 * 4.0 * GEAR1) * M_PI * 2) - M_PI/2.0;
			jnt_pos_[2] = -((double)(pos2 - OFFSET2)/(ENC2 * 4.0 * GEAR2) * M_PI * 2);
			jnt_pos_[3] = -((double)(pos3 - OFFSET3)/(ENC3 * 4.0 * GEAR3) * M_PI * 2);

			jnt_vel_[0] = 0;
			jnt_vel_[1] = 0;
			jnt_vel_[2] = 0;
			jnt_vel_[3] = 0;

			port_JointPosition.write(jnt_pos_);
			port_JointVelocity.write(jnt_vel_);
		} else if(!drivesSynchronized) {
		// Some axes need synchronization. 
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
				std::cout << "All axes synchronized!" << std::endl;
				mc.setMode(0, MODE_CURRENT);
				mc.setMode(1, MODE_POSITION);
				mc.setMode(2, MODE_POSITION);
				mc.setMode(3, MODE_POSITION);
				drivesSynchronized = true;
			}
		} else {
			std::cout << "Some axis un-synchronized?" << std::endl;
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

	bool drivesSynchronized;
	int loopCnt;
	int testMoveDir;

};

ORO_CREATE_COMPONENT(VelmaTorso)

