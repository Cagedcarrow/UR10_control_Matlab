function mdl = create_or_load_virtual_simulink_model(robot, simCfg)
% 创建或加载虚拟UR10 Simulink模型

arguments
    robot (1,1) rigidBodyTree
    simCfg (1,1) struct
end

mdl = simCfg.simModelName;
mdlPath = fullfile(simCfg.baseDir, [mdl '.slx']);

if bdIsLoaded(mdl)
    close_system(mdl, 0);
end

if isfile(mdlPath)
    delete(mdlPath);
end

new_system(mdl);
open_system(mdl);

    add_block('simulink/Sources/From Workspace', [mdl '/q_cmd'], 'Position', [30 85 130 115]);
    set_param([mdl '/q_cmd'], 'VariableName', 'q_cmd_ts');

    add_block('simulink/Discrete/Discrete State-Space', [mdl '/JointSpaceMotionModel'], ...
        'Position', [190 70 430 130]);
    set_param([mdl '/JointSpaceMotionModel'], ...
        'A', '(1-virtual_ur10_alpha)*eye(6)', ...
        'B', 'virtual_ur10_alpha*eye(6)', ...
        'C', 'eye(6)', ...
        'D', 'zeros(6)', ...
        'X0', 'zeros(6,1)');

    add_block('simulink/Sinks/Out1', [mdl '/q_actual'], 'Position', [520 90 550 110]);
    set_param([mdl '/q_actual'], 'PortDimensions', '6');

    add_line(mdl, 'q_cmd/1', 'JointSpaceMotionModel/1');
    add_line(mdl, 'JointSpaceMotionModel/1', 'q_actual/1');

    set_param(mdl, 'StopTime', simCfg.simStopTime, ...
        'SolverType', 'Fixed-step', ...
        'Solver', 'FixedStepDiscrete', ...
        'FixedStep', num2str(simCfg.sampleTime), ...
        'SaveOutput', 'on', ...
        'OutputSaveName', 'yout', ...
        'SignalLogging', 'on', ...
        'SignalLoggingName', 'logsout');

save_system(mdl, mdlPath);

assignin('base','virtual_ur10_robot',robot);
assignin('base','virtual_ur10_cfg',simCfg);
assignin('base','virtual_ur10_alpha',min(1,max(0,simCfg.sampleTime/0.25)));
end
