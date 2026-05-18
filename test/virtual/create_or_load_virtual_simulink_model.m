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

add_block('simulink/Sources/From Workspace', [mdl '/q_exp_ref'], 'Position', [30 195 130 225]);
set_param([mdl '/q_exp_ref'], 'VariableName', 'q_ref_ts');

add_block('simulink/Sources/From Workspace', [mdl '/dq_exp_ref'], 'Position', [30 295 130 325]);
set_param([mdl '/dq_exp_ref'], 'VariableName', 'dq_ref_ts');

add_block('simulink/Sources/From Workspace', [mdl '/tau_exp_csv_ref'], 'Position', [30 395 130 425]);
set_param([mdl '/tau_exp_csv_ref'], 'VariableName', 'tau_csv_ref_ts');

add_block('simulink/Sources/From Workspace', [mdl '/tau_exp_from_current_ref'], 'Position', [30 495 130 525]);
set_param([mdl '/tau_exp_from_current_ref'], 'VariableName', 'tau_from_i_ref_ts');

add_block('simulink/Discrete/Discrete State-Space', [mdl '/JointSpaceMotionModel'], ...
    'Position', [190 70 430 130]);
set_param([mdl '/JointSpaceMotionModel'], ...
    'A', '(1-virtual_ur10_alpha)*eye(6)', ...
    'B', 'virtual_ur10_alpha*eye(6)', ...
    'C', 'eye(6)', ...
    'D', 'zeros(6)', ...
    'X0', 'zeros(6,1)');

add_block('simulink/Discrete/Discrete Derivative', [mdl '/dq_actual'], ...
    'Position', [500 140 600 170]);
set_param([mdl '/dq_actual'], 'gainval', sprintf('1/%g', simCfg.sampleTime), 'ICPrevScaledInput', '0');

add_block('simulink/Discrete/Discrete Derivative', [mdl '/ddq_est'], ...
    'Position', [650 140 750 170]);
set_param([mdl '/ddq_est'], 'gainval', sprintf('1/%g', simCfg.sampleTime), 'ICPrevScaledInput', '0');

add_block('simulink/User-Defined Functions/Interpreted MATLAB Function', [mdl '/estimate_current'], ...
    'Position', [810 95 980 165]);
set_param([mdl '/estimate_current'], 'MATLABFcn', ...
    'estimate_current_simple(u(1:6),u(7:12),u(13:18),virtual_ur10_current_gain_q,virtual_ur10_current_gain_dq,virtual_ur10_current_gain_ddq,virtual_ur10_current_bias)');

add_block('simulink/Signal Routing/Mux', [mdl '/mux_current_input'], ...
    'Position', [770 95 790 165], 'Inputs', '3');

add_block('simulink/Math Operations/Product', [mdl '/tau_est'], ...
    'Position', [1040 110 1100 150]);
set_param([mdl '/tau_est'], 'Multiplication', 'Element-wise(.*)');

add_block('simulink/Sources/Constant', [mdl '/motor_gain'], ...
    'Position', [980 30 1040 60], ...
    'Value', 'virtual_ur10_motor_gains(:)');

add_block('simulink/Sinks/Out1', [mdl '/q_actual_out'], 'Position', [1140 90 1170 110], 'Port', '1');
set_param([mdl '/q_actual_out'], 'PortDimensions', '6');
add_block('simulink/Sinks/Out1', [mdl '/dq_actual_out'], 'Position', [1140 140 1170 160], 'Port', '2');
set_param([mdl '/dq_actual_out'], 'PortDimensions', '6');
add_block('simulink/Sinks/Out1', [mdl '/tau_est_out'], 'Position', [1140 190 1170 210], 'Port', '3');
set_param([mdl '/tau_est_out'], 'PortDimensions', '6');
add_block('simulink/Sinks/Out1', [mdl '/i_est_out'], 'Position', [1140 240 1170 260], 'Port', '4');
set_param([mdl '/i_est_out'], 'PortDimensions', '6');

add_block('simulink/Signal Routing/Mux', [mdl '/mux_q_scope'], ...
    'Position', [680 230 700 310], 'Inputs', '2');
add_block('simulink/Signal Routing/Mux', [mdl '/mux_dq_scope'], ...
    'Position', [680 330 700 410], 'Inputs', '2');
add_block('simulink/Signal Routing/Mux', [mdl '/mux_tau_scope'], ...
    'Position', [680 430 700 510], 'Inputs', '3');

add_block('simulink/Sinks/Scope', [mdl '/scope_q'], 'Position', [780 245 860 295]);
add_block('simulink/Sinks/Scope', [mdl '/scope_dq'], 'Position', [780 345 860 395]);
add_block('simulink/Sinks/Scope', [mdl '/scope_tau'], 'Position', [780 445 860 495]);

add_line(mdl, 'q_cmd/1', 'JointSpaceMotionModel/1');
add_line(mdl, 'JointSpaceMotionModel/1', 'dq_actual/1');
add_line(mdl, 'dq_actual/1', 'ddq_est/1');
add_line(mdl, 'JointSpaceMotionModel/1', 'mux_current_input/1');
add_line(mdl, 'dq_actual/1', 'mux_current_input/2');
add_line(mdl, 'ddq_est/1', 'mux_current_input/3');
add_line(mdl, 'mux_current_input/1', 'estimate_current/1');
add_line(mdl, 'estimate_current/1', 'tau_est/1');
add_line(mdl, 'motor_gain/1', 'tau_est/2');

add_line(mdl, 'JointSpaceMotionModel/1', 'q_actual_out/1');
add_line(mdl, 'dq_actual/1', 'dq_actual_out/1');
add_line(mdl, 'tau_est/1', 'tau_est_out/1');
add_line(mdl, 'estimate_current/1', 'i_est_out/1');

add_line(mdl, 'JointSpaceMotionModel/1', 'mux_q_scope/1');
add_line(mdl, 'q_exp_ref/1', 'mux_q_scope/2');
add_line(mdl, 'dq_actual/1', 'mux_dq_scope/1');
add_line(mdl, 'dq_exp_ref/1', 'mux_dq_scope/2');
add_line(mdl, 'tau_est/1', 'mux_tau_scope/1');
add_line(mdl, 'tau_exp_csv_ref/1', 'mux_tau_scope/2');
add_line(mdl, 'tau_exp_from_current_ref/1', 'mux_tau_scope/3');

add_line(mdl, 'mux_q_scope/1', 'scope_q/1');
add_line(mdl, 'mux_dq_scope/1', 'scope_dq/1');
add_line(mdl, 'mux_tau_scope/1', 'scope_tau/1');

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
assignin('base','virtual_ur10_motor_gains',simCfg.motorGains(:));
assignin('base','virtual_ur10_current_bias',simCfg.currentModel.bias(:));
assignin('base','virtual_ur10_current_gain_q',simCfg.currentModel.kq(:));
assignin('base','virtual_ur10_current_gain_dq',simCfg.currentModel.kdq(:));
assignin('base','virtual_ur10_current_gain_ddq',simCfg.currentModel.kddq(:));
ensure_virtual_from_workspace_signals(simCfg.sampleTime, 2);
end
