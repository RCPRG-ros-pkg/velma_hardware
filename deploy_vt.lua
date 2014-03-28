require "rttlib"
require "rttros"

rttlib.color=true

tc=rtt.getTC()
d=tc:getPeer("Deployer")

d:import("rtt_ros")
d:import("rtt_roscomm")
-- Start of user code imports
ros=rtt.provides("ros")

ros:import("velma_hardware")
ros:import("controller_common")
--ros:import("oro_joint_state_publisher")
-- End of user code

local opttab=utils.proc_args(arg)
local cp=rtt.Variable("ConnPolicy")

function conn2ros(depl, port, topic)
   depl:stream(port,rtt.provides("rostopic"):connection(topic))
end

-------------------------------------------------------------------------------
-- Hardware interface
-------------------------------------------------------------------------------

d:loadComponent("VT", "VelmaTorso")
d:setActivity("VT", 0.001, 6, rtt.globals.ORO_SCHED_RT)
VT = d:getPeer("VT")

VT:configure()

-------------------------------------------------------------------------------
-- Head Control
-------------------------------------------------------------------------------

d:loadComponent("HT", "HeadTrajectory")
d:setActivity("HT", 0.001, 6, rtt.globals.ORO_SCHED_RT)
HT = d:getPeer("HT")

HT:configure()

d:loadComponent("HJ", "HeadTeleopJoy")
d:setActivity("HJ", 0.001, 6, rtt.globals.ORO_SCHED_RT)
HT = d:getPeer("HJ")

HT:configure()

-------------------------------------------------------------------------------
-- Connections and Triggers
-------------------------------------------------------------------------------

-- -- VT.JointPositionCommand is a vector of size 3.
--d:connect("VT.JointPositionCommand", "HT.JointPositionCommand", rtt.Variable("ConnPolicy"))

d:loadComponent("Jc", "VectorConcate2")
d:setActivity("Jc", 0.001, 2, rtt.globals.ORO_SCHED_RT)
Jc = d:getPeer("Jc")
d:connect("HT.FakeJointPositionCommand", "Jc.In0", rtt.Variable("ConnPolicy"))
d:connect("HT.JointPositionCommand", "Jc.In1", rtt.Variable("ConnPolicy"))
d:connect("HJ.FakeJointPositionCommand", "Jc.In0", rtt.Variable("ConnPolicy"))
d:connect("HJ.JointPositionCommand", "Jc.In1", rtt.Variable("ConnPolicy"))
d:connect("Jc.Out", "VT.JointPositionCommand", rtt.Variable("ConnPolicy"))
Jc:configure()

conn2ros(d, "HJ.Joy", "/joy")

Jc:start()

VT:start()
--HT:start()
--HJ:start()


-------------------------------------------------------------------------------
-- ROS Diagnostics
-------------------------------------------------------------------------------


