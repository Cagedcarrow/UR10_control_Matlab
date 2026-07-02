function result = main_mud_smearing_trajectory_fitting(varargin)
%MAIN_MUD_SMEARING_TRAJECTORY_FITTING Compute and animate the mud smearing path.

scriptDir = fileparts(mfilename("fullpath"));
addpath(genpath(scriptDir));

cfg = default_mud_smearing_config();
cfg = applyNameValueOptions(cfg, varargin{:});

[robot, cfg] = load_mud_smearing_robot(cfg);
validate_mud_smearing_model(robot, cfg);

startConfig = initial_mud_smearing_config(robot, cfg);
if cfg.ui.showUi
    result = openMudSmearingGui(robot, cfg, startConfig);
    return;
end

result = computeMudSmearingResult(robot, cfg, startConfig);
printResultSummary(result);

if cfg.visual.showAnimation
    animate_mud_smearing_result(robot, cfg, result.motionConfigs, result.trajectory, result.ik);
end
end

function result = openMudSmearingGui(robot, cfg, startConfig)
result = [];
state = struct();
state.robot = robot;
state.cfg = cfg;
state.startConfig = startConfig;
state.result = [];

fig = figure("Name", "mud smearing trajectory fitting", "Color", "w");
ax = axes("Parent", fig, "Position", [0.08 0.24 0.88 0.70]);

show(robot, startConfig, ...
    "Parent", ax, ...
    "Visuals", "on", ...
    "Collisions", "off", ...
    "Frames", "on", ...
    "PreservePlot", false);
axis(ax, "equal");
grid(ax, "on");
view(ax, 135, 20);
title(ax, "mud smearing trajectory fitting");

calculateButton = uicontrol( ...
    fig, ...
    "Style", "pushbutton", ...
    "Units", "normalized", ...
    "Position", [0.10 0.10 0.22 0.06], ...
    "String", "开始计算", ...
    "Callback", @onCalculate);

fitButton = uicontrol( ...
    fig, ...
    "Style", "pushbutton", ...
    "Units", "normalized", ...
    "Position", [0.34 0.10 0.22 0.06], ...
    "String", "开始拟合", ...
    "Enable", "off", ...
    "Callback", @onFit);

statusLabel = uicontrol( ...
    fig, ...
    "Style", "text", ...
    "Units", "normalized", ...
    "Position", [0.58 0.10 0.36 0.06], ...
    "BackgroundColor", "w", ...
    "HorizontalAlignment", "left", ...
    "String", "点击开始计算生成拟合轨迹。");

    function onCalculate(~, ~)
        calculateButton.Enable = "off";
        fitButton.Enable = "off";
        statusLabel.String = "正在计算轨迹...";
        drawnow;

        try
            state.result = computeMudSmearingResult(state.robot, state.cfg, state.startConfig);
            printResultSummary(state.result);
            setappdata(fig, "mudSmearingResult", state.result);
            statusLabel.String = sprintf("轨迹计算成功：%d 个点，最大位置误差 %.4g m。", ...
                size(state.result.ik.configs, 1), max([state.result.ik.metrics.positionError]));
            fitButton.Enable = "on";
        catch ME
            statusLabel.String = "轨迹计算失败，查看命令行错误。";
            calculateButton.Enable = "on";
            rethrow(ME);
        end
        calculateButton.Enable = "on";
    end

    function onFit(~, ~)
        if isempty(state.result)
            statusLabel.String = "请先点击开始计算。";
            return;
        end

        statusLabel.String = "开始播放拟合动画...";
        drawnow;
        cameraState = captureCameraState(ax);
        animate_mud_smearing_result(state.robot, state.cfg, ...
            state.result.motionConfigs, state.result.trajectory, state.result.ik, cameraState);
        statusLabel.String = "拟合动画播放完成。";
    end
end

function result = computeMudSmearingResult(robot, cfg, startConfig)
trajectory = build_wall_contact_trajectory(robot, startConfig, cfg);
ikResult = solve_mud_smearing_ik(robot, cfg, startConfig, trajectory);
motionConfigs = interpolate_joint_path(startConfig, ikResult.configs(1, :), ...
    cfg.trajectory.approachSteps);
motionConfigs = [motionConfigs; ikResult.configs(2:end, :)];

result = struct();
result.config = cfg;
result.robot = robot;
result.startConfig = startConfig;
result.trajectory = trajectory;
result.ik = ikResult;
result.motionConfigs = motionConfigs;
end

function printResultSummary(result)
fprintf("Mud smearing IK solved: %d waypoints, max position error %.4f m, max center-axis angle %.2f deg.\n", ...
    size(result.ik.configs, 1), ...
    max([result.ik.metrics.positionError]), ...
    max([result.ik.metrics.maxCenterAxisAngleDeg]));
end

function cameraState = captureCameraState(ax)
cameraState = struct();
cameraState.CameraPosition = ax.CameraPosition;
cameraState.CameraTarget = ax.CameraTarget;
cameraState.CameraUpVector = ax.CameraUpVector;
cameraState.CameraViewAngle = ax.CameraViewAngle;
cameraState.Projection = ax.Projection;
end

function cfg = applyNameValueOptions(cfg, varargin)
if mod(numel(varargin), 2) ~= 0
    error("Options must be name-value pairs.");
end

for idx = 1:2:numel(varargin)
    name = lower(string(varargin{idx}));
    value = varargin{idx + 1};
    switch name
        case "showanimation"
            cfg.visual.showAnimation = logical(value);
        case "showui"
            cfg.ui.showUi = logical(value);
        case "numwaypoints"
            cfg.trajectory.numWaypoints = value;
        otherwise
            error("Unknown option: %s", varargin{idx});
    end
end
end
