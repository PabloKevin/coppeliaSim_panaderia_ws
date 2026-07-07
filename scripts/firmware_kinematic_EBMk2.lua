-- ============================================================
--  EEZYbotARM Mk2 Kinematic Linkage
--  Attach as a NON-THREADED child script on /EZZYbotARM_Mk2
--
--  All joints and links are KINEMATIC (not dynamic).
--  This script:
--    . Enforces the 4-bar parallel linkage by setting all
--      passive auxJoint positions every simulation step.
--
--  JOINT MODES REQUIRED IN COPPELIA:
--    motor1, motor2, motor3    Kinematic, position control
--    auxMotor1, auxMotor2      Kinematic, position control
--    all passive auxJoints     Kinematic, free (or position)
--    all links/meshes          NOT dynamic, respondable
-- ============================================================

sim = require 'sim'

-- Joint handles populated in sysCall_init
local motor1    = nil
local motor2    = nil
local motor3    = nil

-- Upper loop passive joints
local auxJoint              = nil   -- 001_base_visual ? 005_link_135angled
local motor3_auxJoint       = nil   -- 003_varm_visual ? 004_link_135_visual
local link135_auxJoint      = nil   -- 004_link_135_visual ? loopDummy2A
local link135angled_auxJoint = nil  -- 005_link_135angled_respondable ? loopDummy1A

-- Lower loop joints
local auxMotor1             = nil   -- 002_mainarm_visual ? 006_horarm_visual
local motor2_auxJoint       = nil   -- auxMotor1 ? 007_trialink_visual
local auxMotor2             = nil   -- 006_horarm_visual ? 009_trialinkfront_visual
local horarm_auxJoint       = nil   -- 009_trialinkfront_visual ? 008_link_147_visual


-- ============================================================
function sysCall_init()
    motor1 = sim.getObject('../motor1')
    motor2 = sim.getObject('../motor2')
    motor3 = sim.getObject('../motor3')

    -- Upper loop
    auxJoint               = sim.getObject('../auxJoint')
    motor3_auxJoint        = sim.getObject('../motor3/auxJoint')
    link135_auxJoint       = sim.getObject('../004_link_135_visual_convexDecomposed/auxJoint')
    link135angled_auxJoint = sim.getObject('../auxJoint/auxJoint')

    -- Lower loop
    auxMotor1              = sim.getObject('../auxMotor1')
    motor2_auxJoint        = sim.getObject('../motor2/auxJoint')
    auxMotor2              = sim.getObject('../auxMotor2')
    horarm_auxJoint        = sim.getObject('../006_horarm_visual_convexDecomposed/auxJoint')
    
    sim.addLog(sim.verbosity_infos,
        '[EBA_MK2] Linkage + limits script initialised OK.')
end

-- ============================================================
function sysCall_actuation()

    -- 1. Read current motor positions
    local t1 = sim.getJointPosition(motor1)
    local t2 = sim.getJointPosition(motor2)
    local t3 = sim.getJointPosition(motor3)

    -- 3. Upper 4-bar loop 
    -- Transfers elbow rotation through the angled bars so the
    -- forearm attachment point stays geometrically consistent.
    sim.setJointPosition(auxJoint,               t2)
    sim.setJointPosition(motor3_auxJoint,        t2 + t3)
    sim.setJointPosition(link135_auxJoint,        t3)   -- loop closure
    sim.setJointPosition(link135angled_auxJoint,  t3)   -- loop closure

    -- 4. Lower 4-bar loop 
    -- Keeps the wrist platform (009_trialinkfront_visual) level
    -- regardless of shoulder and elbow movement.
    sim.setJointPosition(auxMotor1,       t3 + t2)
    sim.setJointPosition(motor2_auxJoint, -t3)
    sim.setJointPosition(auxMotor2,       -t3)
    sim.setJointPosition(horarm_auxJoint, -t3)      -- loop closure

end