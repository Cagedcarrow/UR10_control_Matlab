function varargout = rtfg_render(action, varargin)
switch action
    case 'refreshScene'
        varargout{1} = refreshScene(varargin{1}, varargin{2});
    case 'renderScene'
        varargout{1} = renderScene(varargin{1});
    case 'captureCameraState'
        varargout{1} = captureCameraState(varargin{1}, varargin{2});
    case 'getPresetView'
        varargout{1} = getPresetView(varargin{1});
    otherwise
        error('rtfg_render:UnknownAction', 'Unknown action: %s', action);
end
end

function state = refreshScene(state, usePresetView)
state.envGeom.basin.mudSurfaceZ = state.envGeom.basin.floorZ + state.trajParams.mudHeight;
state.traj = generate_trajectory_3d(state.trajParams, state.envGeom);
previewText = rtfg_io('buildPreviewUrdfText', state.paths.sceneUrdf, state.pose);
rtfg_io('writeTextFile', state.tempUrdfPath, previewText);
state.robot = importrobot(state.tempUrdfPath, ...
    'DataFormat', 'row', ...
    'MeshPath', state.paths.meshDir);
state.collisionRobot = importrobot(state.paths.collisionRobotUrdf, ...
    'DataFormat', 'row', ...
    'MeshPath', state.paths.meshDir);
state.collisionEnv = rtfg_collision('buildEnvironment', state);
state.ikSolver = [];

if isempty(state.currentQ) || numel(state.currentQ) ~= numel(state.initialJointPosition)
    state.currentQ = state.initialJointPosition;
end

if usePresetView
    presetName = getUiDropdownValue(state.ui, 'ddView', "默认");
    state.ui.cameraState = struct('mode', 'preset', 'value', getPresetView(presetName));
elseif isfield(state.ui, 'mainAx') && isgraphics(state.ui.mainAx)
    state.ui.cameraState = captureCameraState(state.ui.mainAx, state.ui.cameraState);
end
end

function state = renderScene(state)
ax = state.ui.mainAx;
cla(ax);
showFrames = getUiToggleValue(state.ui, 'cbFrames', true);
showCollisionMarkers = getUiToggleValue(state.ui, 'cbCollisionMarkers', true);
showWorld = getUiToggleValue(state.ui, 'cbWorld', true);
showTrajectoryArrows = getUiToggleValue(state.ui, 'cbTrajectoryArrows', true);
show(state.robot, state.currentQ, ...
    'Parent', ax, ...
    'Visuals', 'on', ...
    'Collisions', 'off', ...
    'Frames', rtfg_utils('ternary', showFrames, 'on', 'off'), ...
    'PreservePlot', false, ...
    'FastUpdate', true);
hold(ax, 'on');

traj = rtfg_utils('applyPoseToTrajectory', state.traj, state.pose);
envGeom = state.envGeom;

plot3(ax, traj.all(:, 1), traj.all(:, 2), traj.all(:, 3), '.-', ...
    'Color', [0.12 0.45 0.95], 'LineWidth', 1.6, 'MarkerSize', 10);
plot3(ax, traj.pApproachStart(1), traj.pApproachStart(2), traj.pApproachStart(3), 's', ...
    'Color', [0.25 0.25 0.25], 'MarkerFaceColor', [0.85 0.85 0.85], 'MarkerSize', 6);
plot3(ax, traj.pEntry(1), traj.pEntry(2), traj.pEntry(3), 'ko', ...
    'MarkerFaceColor', 'k', 'MarkerSize', 6);
plot3(ax, traj.pArcEnd(1), traj.pArcEnd(2), traj.pArcEnd(3), 'o', ...
    'Color', [0.85 0.25 0.25], 'MarkerFaceColor', [0.85 0.25 0.25], 'MarkerSize', 6);

drawPlanePatch(ax, envGeom.basin, state.traj.xPlane, state.pose);
drawMudSurfacePatch(ax, envGeom.basin, state.pose);
drawMudSurfaceLine(ax, envGeom.basin, state.traj.xPlane, state.pose);
if ~isempty(state.previewTcpPath)
    plot3(ax, state.previewTcpPath(:, 1), state.previewTcpPath(:, 2), state.previewTcpPath(:, 3), '-', ...
        'Color', [0.84 0.10 0.65], 'LineWidth', 1.8);
end
if ~isempty(state.previewTargetZAxes)
    drawPreviewOrientationAxes(ax, state.previewTargetZAxes);
end
if showCollisionMarkers
    drawCollisionResults(ax, state);
end
tcpNow = rtfg_utils('getTcpPosition', state.robot, state.currentQ);
plot3(ax, tcpNow(1), tcpNow(2), tcpNow(3), 'o', ...
    'Color', [0.95 0.15 0.15], 'MarkerFaceColor', [0.95 0.15 0.15], 'MarkerSize', 7);

if showWorld
    drawWorldAxes(ax);
end
if showTrajectoryArrows
    drawTrajectoryArrows(ax, traj);
end

axis(ax, 'equal');
axis(ax, 'vis3d');
grid(ax, 'on');
xlabel(ax, 'X (m)');
ylabel(ax, 'Y (m)');
zlabel(ax, 'Z (m)');
xlim(ax, [-2.4, 0.8]);
ylim(ax, [-0.9, 0.9]);
zlim(ax, [-0.4, 1.8]);
applyCameraState(ax, state.ui.cameraState);
title(ax, sprintf(['环境 + 铲泥轨迹 | x = %.3f m | 距 X- = %.3f m | 距 X+ = %.3f m', newline, ...
    '距左侧壁 = %.3f m | hMud = %.0f mm | L = %.3f m | \\theta = %.1f deg | d = %.3f m'], ...
    traj.xPlane, traj.distLeft, traj.distRight, ...
    traj.leftWallOffset, 1000 * (traj.mudSurfaceZ - envGeom.basin.floorZ), ...
    traj.approachLen, traj.thetaDeg, traj.depth));
drawnow limitrate nocallbacks;
hold(ax, 'off');
end

function value = getUiToggleValue(uiStruct, fieldName, defaultValue)
value = defaultValue;
if ~isfield(uiStruct, fieldName)
    return;
end
handle = uiStruct.(fieldName);
if isempty(handle)
    return;
end
if isgraphics(handle) || isvalid(handle)
    try
        value = logical(handle.Value);
    catch
        value = defaultValue;
    end
end
end

function value = getUiDropdownValue(uiStruct, fieldName, defaultValue)
value = defaultValue;
if ~isfield(uiStruct, fieldName)
    return;
end
handle = uiStruct.(fieldName);
if isempty(handle)
    return;
end
if isgraphics(handle) || isvalid(handle)
    try
        value = string(handle.Value);
    catch
        value = defaultValue;
    end
end
end

function drawCollisionResults(ax, state)
results = state.collisionResults;
if isempty(results) || ~isstruct(results) || ~isfield(results, 'hasCollision') || ~results.hasCollision
    return;
end

plot3(ax, ...
    results.collisionPoints(:, 1), results.collisionPoints(:, 2), results.collisionPoints(:, 3), 'o', ...
    'Color', [0.95 0.10 0.10], ...
    'MarkerFaceColor', [0.95 0.10 0.10], ...
    'MarkerSize', 5);

if ~isempty(results.firstCollision)
    p = results.firstCollision.point;
    plot3(ax, p(1), p(2), p(3), 'o', ...
        'Color', [1.0 0.0 0.0], ...
        'MarkerFaceColor', [1.0 0.92 0.2], ...
        'MarkerSize', 9, ...
        'LineWidth', 1.5);
end

if isfield(state, 'collisionEnv') && isfield(state.collisionEnv, 'basinBoxes')
    hitNames = string(results.hitBasinNames);
    for i = 1:numel(state.collisionEnv.basinBoxes)
        boxSpec = state.collisionEnv.basinBoxes(i);
        if any(hitNames == string(boxSpec.name))
            drawHighlightedBox(ax, boxSpec.poseWorld, boxSpec.size);
        end
    end
end
end

function drawHighlightedBox(ax, poseWorld, boxSize)
vertsLocal = 0.5 * [
    -boxSize(1), -boxSize(2), -boxSize(3)
    boxSize(1), -boxSize(2), -boxSize(3)
    boxSize(1), boxSize(2), -boxSize(3)
    -boxSize(1), boxSize(2), -boxSize(3)
    -boxSize(1), -boxSize(2), boxSize(3)
    boxSize(1), -boxSize(2), boxSize(3)
    boxSize(1), boxSize(2), boxSize(3)
    -boxSize(1), boxSize(2), boxSize(3)];
vertsWorld = (poseWorld(1:3, 1:3) * vertsLocal.' + poseWorld(1:3, 4)).';
edges = [
    1 2; 2 3; 3 4; 4 1
    5 6; 6 7; 7 8; 8 5
    1 5; 2 6; 3 7; 4 8];
for i = 1:size(edges, 1)
    pts = vertsWorld(edges(i, :), :);
    plot3(ax, pts(:, 1), pts(:, 2), pts(:, 3), '-', ...
        'Color', [0.95 0.10 0.10], ...
        'LineWidth', 2.0);
end
end

function drawTrajectoryArrows(ax, traj)
idx = unique(round(linspace(1, size(traj.all, 1), 14)));
arrowLen = 0.05;
dirs = arrowLen * traj.tangent3d(idx, :);
quiver3(ax, ...
    traj.all(idx, 1), traj.all(idx, 2), traj.all(idx, 3), ...
    dirs(:, 1), dirs(:, 2), dirs(:, 3), 0, ...
    'Color', [0.18 0.18 0.18], 'LineWidth', 1.0, 'MaxHeadSize', 1.4);
end

function drawPlanePatch(ax, basin, xPlane, pose)
pts = [
    xPlane, basin.innerYMin, basin.floorZ
    xPlane, basin.innerYMax, basin.floorZ
    xPlane, basin.innerYMax, basin.rimZ
    xPlane, basin.innerYMin, basin.rimZ];
pts = rtfg_utils('transformPointsByPose', pts, pose);
patch(ax, pts(:, 1), pts(:, 2), pts(:, 3), [0.95 0.70 0.10], ...
    'FaceAlpha', 0.06, 'EdgeColor', [0.85 0.55 0.10], 'LineStyle', '--', 'LineWidth', 1.0);
end

function drawMudSurfacePatch(ax, basin, pose)
pts = [
    basin.innerXMin, basin.innerYMin, basin.mudSurfaceZ
    basin.innerXMax, basin.innerYMin, basin.mudSurfaceZ
    basin.innerXMax, basin.innerYMax, basin.mudSurfaceZ
    basin.innerXMin, basin.innerYMax, basin.mudSurfaceZ];
pts = rtfg_utils('transformPointsByPose', pts, pose);
patch(ax, pts(:, 1), pts(:, 2), pts(:, 3), [0.97 0.90 0.55], ...
    'FaceAlpha', 0.32, 'EdgeColor', [0.85 0.74 0.22], 'LineWidth', 1.0);
end

function drawMudSurfaceLine(ax, basin, xPlane, pose)
pts = [
    xPlane, basin.innerYMin, basin.mudSurfaceZ
    xPlane, basin.innerYMax, basin.mudSurfaceZ];
pts = rtfg_utils('transformPointsByPose', pts, pose);
plot3(ax, pts(:, 1), pts(:, 2), pts(:, 3), '--', ...
    'Color', [0.80 0.62 0.12], 'LineWidth', 1.4);
end

function drawWorldAxes(ax)
origin = [0.0, 0.0, 0.0];
len = 0.18;
quiver3(ax, origin(1), origin(2), origin(3), len, 0, 0, 0, ...
    'Color', [0.90 0.15 0.10], 'LineWidth', 1.7, 'MaxHeadSize', 0.7);
quiver3(ax, origin(1), origin(2), origin(3), 0, len, 0, 0, ...
    'Color', [0.10 0.65 0.25], 'LineWidth', 1.7, 'MaxHeadSize', 0.7);
quiver3(ax, origin(1), origin(2), origin(3), 0, 0, len, 0, ...
    'Color', [0.15 0.35 0.95], 'LineWidth', 1.7, 'MaxHeadSize', 0.7);
text(ax, len * 1.1, 0, 0, 'X', 'Color', [0.90 0.15 0.10], 'FontWeight', 'bold');
text(ax, 0, len * 1.1, 0, 'Y', 'Color', [0.10 0.65 0.25], 'FontWeight', 'bold');
text(ax, 0, 0, len * 1.1, 'Z', 'Color', [0.15 0.35 0.95], 'FontWeight', 'bold');
end

function drawPreviewOrientationAxes(ax, zAxisPreview)
if isempty(zAxisPreview)
    return;
end
arrowLen = 0.05;
quiver3(ax, ...
    zAxisPreview(:, 1), zAxisPreview(:, 2), zAxisPreview(:, 3), ...
    arrowLen * zAxisPreview(:, 4), arrowLen * zAxisPreview(:, 5), arrowLen * zAxisPreview(:, 6), 0, ...
    'Color', [0.95 0.45 0.05], 'LineWidth', 1.1, 'MaxHeadSize', 1.1);
end

function cameraState = captureCameraState(ax, fallbackState)
try
    cameraState = struct( ...
        'mode', 'camera', ...
        'position', ax.CameraPosition, ...
        'target', ax.CameraTarget, ...
        'upVector', ax.CameraUpVector, ...
        'viewAngle', ax.CameraViewAngle);
catch
    cameraState = fallbackState;
end
end

function applyCameraState(ax, cameraState)
if isempty(cameraState) || ~isstruct(cameraState) || ~isfield(cameraState, 'mode')
    view(ax, 135, 20);
    return;
end

switch cameraState.mode
    case 'preset'
        view(ax, cameraState.value(1), cameraState.value(2));
    case 'camera'
        ax.CameraPosition = cameraState.position;
        ax.CameraTarget = cameraState.target;
        ax.CameraUpVector = cameraState.upVector;
        ax.CameraViewAngle = cameraState.viewAngle;
    otherwise
        view(ax, 135, 20);
end
end

function azEl = getPresetView(viewName)
switch viewName
    case 'Top'
        azEl = [0, 90];
    case 'Front'
        azEl = [0, 0];
    case 'Side'
        azEl = [90, 0];
    otherwise
        azEl = [135, 20];
end
end
