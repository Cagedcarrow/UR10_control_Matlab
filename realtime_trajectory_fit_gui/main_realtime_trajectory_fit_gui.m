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
state.previewQSeries = [];
state.previewTcpPath = [];
state.previewTargetZAxes = [];
state.ikSolver = [];
state.isAnimating = false;
state.robot = [];
state.traj = [];
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
        refreshScene(false);
    end

    function onPoseParamChanged(fieldName, value)
        state.pose.(fieldName) = value;
        invalidatePreviewResults();
        state = rtfg_ui('updateSummaries', state);
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
        catch ME
            state = rtfg_ui('setStatus', state, sprintf('刷新失败: %s', ME.message));
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
        refreshScene(false);
        state = rtfg_ui('setStatus', state, '已从 YAML 重新加载配置。');
    end

    function onSaveYaml()
        rtfg_io('writeRuntimeConfigYaml', state.paths.runtimeYaml, state);
        state = rtfg_ui('setStatus', state, sprintf('YAML 已保存: %s', state.paths.runtimeYaml));
    end

    function onSaveYamlAndUrdf()
        rtfg_io('writeRuntimeConfigYaml', state.paths.runtimeYaml, state);
        rtfg_io('writePoseToUrdf', state.paths.sceneUrdf, state.pose);
        state = rtfg_ui('setStatus', state, 'YAML 和 URDF 都已更新。');
    end

    function onMoveToTrajectoryStart()
        if state.isAnimating
            state = rtfg_ui('setStatus', state, '当前正在播放预览，请等待结束。');
            return;
        end
        try
            [state, statusText] = rtfg_kinematics('moveToTrajectoryStart', state);
            renderCombinedScene();
            state = rtfg_ui('setStatus', state, statusText);
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
            [state, statusText] = rtfg_kinematics('trackTrajectory', state);
            renderCombinedScene();
            state = rtfg_ui('setStatus', state, statusText);
        catch ME
            showKinematicsFailure('尖端轨迹拟合失败', ME);
        end
    end

    function invalidatePreviewResults()
        state.previewQSeries = [];
        state.previewTcpPath = [];
        state.previewTargetZAxes = [];
    end

    function showKinematicsFailure(titleText, ME)
        invalidatePreviewResults();
        renderCombinedScene();
        msg = sprintf('%s: %s', titleText, ME.message);
        state = rtfg_ui('setStatus', state, msg);
        uialert(state.ui.mainFig, msg, titleText);
    end
end
