function out = startup_virtual_ur10()
% 虚拟UR10系统一键启动入口（不依赖实机）

baseDir = fileparts(mfilename('fullpath'));
addpath(baseDir);
addpath(fullfile(baseDir,'utils'));

cfg = init_virtual_ur10(baseDir);
ensure_virtual_from_workspace_signals(cfg.sampleTime, 2);
[robot, robotCfg] = build_ur10_virtual_robot(cfg);
mdl = create_or_load_virtual_simulink_model(robot, cfg);

assignin('base','virtual_ur10_cfg',cfg);
assignin('base','virtual_ur10_robot',robot);
assignin('base','virtual_ur10_robot_cfg',robotCfg);
assignin('base','virtual_ur10_model',mdl);

ui = ur10_gui_control_virtual(robot, cfg, mdl); %#ok<NASGU>

out = struct('cfg',cfg,'robot',robot,'robotCfg',robotCfg,'model',mdl,'gui',ui);
fprintf('[INFO] 虚拟UR10系统已启动。\n');
fprintf('[INFO] 模型: %s\n', mdl);
fprintf('[INFO] 可运行示例: run_virtual_trajectory_demo(robot, mdl, cfg)\n');
end
