function config = initial_mud_smearing_config(robot, cfg)
%INITIAL_MUD_SMEARING_CONFIG Return the configured initial 7-DOF pose.

config = homeConfiguration(robot);
movableJointIndex = 0;

for bodyIndex = 1:robot.NumBodies
    joint = robot.Bodies{bodyIndex}.Joint;
    if strcmp(joint.Type, "fixed")
        continue;
    end

    movableJointIndex = movableJointIndex + 1;
    jointName = char(joint.Name);
    if isfield(cfg.initialJointValues, jointName)
        config(movableJointIndex) = cfg.initialJointValues.(jointName);
    end
end
end
