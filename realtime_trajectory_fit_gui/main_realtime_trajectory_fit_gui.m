function main_realtime_trajectory_fit_gui()
%MAIN_REALTIME_TRAJECTORY_FIT_GUI Unified environment + trajectory preview GUI.
%
% This GUI combines:
%   1) block_with_basin environment pose adjustment
%   2) 3D scoop trajectory parameter adjustment
%   3) unified YAML save/reload
%   4) optional URDF pose write-back

thisDir = fileparts(mfilename('fullpath'));
addpath(fullfile(thisDir, 'ui'));
addpath(fullfile(thisDir, 'rendering'));
addpath(fullfile(thisDir, 'kinematics'));
addpath(fullfile(thisDir, 'trajectory'));
addpath(fullfile(thisDir, 'io'));
addpath(fullfile(thisDir, 'utils'));
addpath(fullfile(thisDir, 'collision'));
addpath(fullfile(thisDir, 'mex'));
addpath(fullfile(thisDir, 'mex', 'bin'));

paths = rtfg_io('buildPaths', thisDir);
rtfg_io('ensureRequiredPaths', paths);
addpath(paths.trajectoryGuiDir);

baseState = build_gui_state('trajectory_params_3d.yaml');
state = struct();
state.paths = paths;
state.envGeom = baseState.envGeom;
state.trajParams = baseState.params;
state.pose = rtfg_io('loadFallbackPose', paths);
state.initialJointPosition = rtfg_io('readInitialJointPosition', paths.sceneUrdf);
state.tempUrdfPath = fullfile(tempdir, 'realtime_trajectory_fit_preview.urdf');
state.currentQ = state.initialJointPosition;
state.previewAnchorQSeries = [];
state.previewQSeries = [];
state.previewTcpPath = [];
state.previewTargetZAxes = [];
state.previewMetrics = struct();
state.ikSolver = [];
state.isAnimating = false;
state.robot = [];
state.collisionRobot = [];
state.traj = [];
state.collisionEnabled = true;
state.collisionConfig = rtfg_collision('buildConfig');
state.collisionEnv = struct();
state.collisionResults = rtfg_collision('emptyResults');
state.ui = struct();
state.ui.cameraState = struct('mode', 'preset', 'value', [135, 20]);

if isfile(paths.runtimeYaml)
    [state.trajParams, state.pose] = rtfg_io('readRuntimeConfigYaml', paths.runtimeYaml, state.trajParams, state.pose);
end

callbacks = buildUiCallbacks();
state = rtfg_ui('buildMainUi', state, callbacks);
refreshScene(true);

    function callbacks = buildUiCallbacks()
        callbacks = struct( ...
            'openTrajectoryWindow', @openTrajectoryWindow, ...
            'openEnvironmentWindow', @openEnvironmentWindow, ...
            'moveToTrajectoryStart', @onMoveToTrajectoryStart, ...
            'trackTcpTrajectory', @onTrackTcpTrajectory, ...
            'playTcpTrajectory', @onPlayTcpTrajectory, ...
            'reloadYaml', @onReloadYaml, ...
            'saveYaml', @onSaveYaml, ...
            'saveYamlAndUrdf', @onSaveYamlAndUrdf, ...
            'presetViewChanged', @onPresetViewChanged, ...
            'refreshScene', @refreshScene, ...
            'trajParamChanged', @onTrajectoryParamChanged, ...
            'poseParamChanged', @onPoseParamChanged, ...
            'childClosed', @onChildClosed);
    end

    function openTrajectoryWindow()
        state = rtfg_ui('openTrajectoryWindow', state, callbacks);
    end

    function openEnvironmentWindow()
        state = rtfg_ui('openEnvironmentWindow', state, callbacks);
    end

    function onChildClosed(fieldName)
        state.ui.(fieldName) = [];
    end

    function onTrajectoryParamChanged(fieldName, value)
        state.trajParams.(fieldName) = value;
        invalidatePreviewResults();
        state = rtfg_ui('updateSummaries', state);
        state = rtfg_ui('setRunEnabled', state, false);
        refreshScene(false);
    end

    function onPoseParamChanged(fieldName, value)
        state.pose.(fieldName) = value;
        invalidatePreviewResults();
        state = rtfg_ui('updateSummaries', state);
        state = rtfg_ui('setRunEnabled', state, false);
        refreshScene(false);
    end

    function onPresetViewChanged()
        state.ui.cameraState = struct('mode', 'preset', 'value', rtfg_render('getPresetView', state.ui.ddView.Value));
        refreshScene(true);
    end

    function refreshScene(usePresetView)
        try
            state = rtfg_render('refreshScene', state, usePresetView);
            renderCombinedScene();
            updateSummaries();
            state = rtfg_ui('setStatus', state, '已加载。');
            state = rtfg_ui('setProgress', state, 0, '待机');
        catch ME
            state = rtfg_ui('setStatus', state, sprintf('刷新失败: %s', ME.message));
            state = rtfg_ui('setProgress', state, 0, '刷新失败');
        end
    end

    function renderCombinedScene()
        state = rtfg_render('renderScene', state);
    end

    function updateSummaries()
        state = rtfg_ui('updateSummaries', state);
    end

    function onReloadYaml()
        if ~isfile(state.paths.runtimeYaml)
            uialert(state.ui.mainFig, sprintf('YAML 文件不存在:\n%s', state.paths.runtimeYaml), '读取失败');
            return;
        end
        [state.trajParams, state.pose] = rtfg_io('readRuntimeConfigYaml', state.paths.runtimeYaml, state.trajParams, state.pose);
        invalidatePreviewResults();
        state = rtfg_ui('setRunEnabled', state, false);
        refreshScene(false);
        state = rtfg_ui('setStatus', state, '已从 YAML 重新加载配置。');
        state = rtfg_ui('setProgress', state, 0, '待机');
    end

    function onSaveYaml()
        rtfg_io('writeRuntimeConfigYaml', state.paths.runtimeYaml, state);
        state = rtfg_ui('setStatus', state, sprintf('YAML 已保存: %s', state.paths.runtimeYaml));
        state = rtfg_ui('setProgress', state, 0, '待机');
    end

    function onSaveYamlAndUrdf()
        rtfg_io('writeRuntimeConfigYaml', state.paths.runtimeYaml, state);
        rtfg_io('writePoseToUrdf', state.paths.sceneUrdf, state.pose);
        state = rtfg_ui('setStatus', state, 'YAML 和 URDF 都已更新。');
        state = rtfg_ui('setProgress', state, 0, '待机');
    end

    function onMoveToTrajectoryStart()
        if state.isAnimating
            state = rtfg_ui('setStatus', state, '当前正在播放预览，请等待结束。');
            return;
        end
        try
            state = rtfg_ui('setStatus', state, '正在计算移动到轨迹起始点，请稍候...');
            state = rtfg_ui('setProgress', state, 0, '移动到起始点');
            [state, statusText] = rtfg_kinematics('moveToTrajectoryStart', state);
            renderCombinedScene();
            state = rtfg_ui('setStatus', state, statusText);
            state = rtfg_ui('setProgress', state, 1, '移动到起始点完成');
        catch ME
            showKinematicsFailure('移动到起始点失败', ME);
        end
    end

    function onTrackTcpTrajectory()
        if state.isAnimating
            state = rtfg_ui('setStatus', state, '当前正在播放预览，请等待结束。');
            return;
        end
        try
            state = rtfg_ui('setStatus', state, '正在进行尖端轨迹拟合，请稍候...');
            state = rtfg_ui('setProgress', state, 0, '尖端轨迹拟合');
            [state, statusText] = rtfg_kinematics('trackTrajectory', state);
            renderCombinedScene();
            state = rtfg_ui('setStatus', state, statusText);
            state = rtfg_ui('setProgress', state, 1, '尖端轨迹拟合完成');
            state = rtfg_ui('setRunEnabled', state, true);
        catch ME
            showKinematicsFailure('尖端轨迹拟合失败', ME);
        end
    end

    function onPlayTcpTrajectory()
        if state.isAnimating
            state = rtfg_ui('setStatus', state, '当前正在播放预览，请等待结束。');
            return;
        end
        if isempty(state.previewQSeries)
            state = rtfg_ui('setStatus', state, '请先完成尖端轨迹拟合，再点击开始运行。');
            return;
        end
        try
            state = rtfg_ui('setStatus', state, '正在播放已拟合轨迹，请稍候...');
            state = rtfg_ui('setProgress', state, 0, '开始运行');
            [state, statusText] = rtfg_kinematics('playPreviewTrajectory', state);
            renderCombinedScene();
            state = rtfg_ui('setStatus', state, statusText);
            state = rtfg_ui('setProgress', state, 1, '运行完成');
        catch ME
            showKinematicsFailure('运行拟合轨迹失败', ME);
        end
    end

    function invalidatePreviewResults()
        state.previewAnchorQSeries = [];
        state.previewQSeries = [];
        state.previewTcpPath = [];
        state.previewTargetZAxes = [];
        state.previewMetrics = struct();
        state.collisionResults = rtfg_collision('emptyResults');
        state = rtfg_ui('setRunEnabled', state, false);
    end

    function showKinematicsFailure(titleText, ME)
        invalidatePreviewResults();
        renderCombinedScene();
        msg = sprintf('%s: %s', titleText, ME.message);
        state = rtfg_ui('setStatus', state, msg);
        state = rtfg_ui('setProgress', state, 0, titleText);
        uialert(state.ui.mainFig, msg, titleText);
    end
end
