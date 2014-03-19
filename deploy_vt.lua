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

-------------------------------------------------------------------------------
-- Connections and Triggers
-------------------------------------------------------------------------------

d:connect("VT.JointPositionCommand", "HT.JointPositionCommand", rtt.Variable("ConnPolicy"))

VT:start()
HT:start()


-------------------------------------------------------------------------------
-- ROS Diagnostics
-------------------------------------------------------------------------------


