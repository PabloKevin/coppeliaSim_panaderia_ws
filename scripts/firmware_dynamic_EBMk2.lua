-- ============================================================
--  EEZYbotARM Mk2 Dynamic Gripper
-- ============================================================

sim = require 'sim'

-- Joint handles populated in sysCall_init
local motor4    = nil
local auxJoint4    = nil
local auxJoint_r    = nil
local auxJoint_l    = nil

-- ============================================================
function sysCall_init()
    motor4 = sim.getObject('../motor4')
    auxJoint4 = sim.getObject('../auxJoint4')
    auxJoint_r = sim.getObject('../auxJoint_r')
    auxJoint_l = sim.getObject('../auxJoint_l')
    
    sim.addLog(sim.verbosity_infos,
        '[EBA_MK2] Gripper script initialised OK.')
end

-- ============================================================
function sysCall_actuation()

    -- 1. Read current motor position
    local t1 = sim.getJointPosition(motor4)

    -- 2. Set the target position of the auxiliary joints based on the motor position
    sim.setJointTargetPosition(auxJoint4, -t1)
    sim.setJointTargetPosition(auxJoint_r, -t1)
    sim.setJointTargetPosition(auxJoint_l, t1)
end