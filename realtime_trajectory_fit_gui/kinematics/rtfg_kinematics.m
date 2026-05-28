function varargout = rtfg_kinematics(action, varargin)
switch action
    case 'moveToTrajectoryStart'
        [varargout{1}, varargout{2}] = moveToTrajectoryStart(varargin{1});
    case 'trackTrajectory'
        [varargout{1}, varargout{2}] = trackTrajectory(varargin{1});
    otherwise
        error('rtfg_kinematics:UnknownAction', 'Unknown action: %s', action);
end
end

function [state, statusText] = moveToTrajectoryStart(state)
targetPlan = buildTrajectoryTargetPlan(getDisplayTrajectory(state));
targetTform = targetPlan.tforms{1};
state.ikSolver = ensureIkSolver(state.robot);
seedList = buildIkSeedList(state.currentQ, state.initialJointPosition);
[qStart, info] = solveTcpPoseIk(state.ikSolver, targetTform, seedList);
qSeries = rtfg_quintic_joint_series(state.currentQ, qStart, 70);
state.previewQSeries = qSeries;
state.ui.cameraState = captureCurrentCameraState(state);
state = playPreviewJointSeries(state, qSeries);
state.previewTcpPath = rtfg_utils('computeTcpPath', state.robot, qSeries);
state.previewTargetZAxes = targetPlan.zAxisPreview;
state.currentQ = qStart;
statusText = sprintf('已到达轨迹起始点。IK 退出标志: %s', rtfg_utils('stringifyIkStatus', info));
end

function [state, statusText] = trackTrajectory(state)
targetPlan = buildTrajectoryTargetPlan(getDisplayTrajectory(state));
state.ikSolver = ensureIkSolver(state.robot);
sampledPlan = rtfg_downsample_target_plan_keep_boundaries(targetPlan, 140);
qSeries = zeros(numel(sampledPlan.tforms), numel(state.currentQ));
qSeed = state.currentQ;
for i = 1:numel(sampledPlan.tforms)
    targetTform = sampledPlan.tforms{i};
    try
        seedList = buildIkSeedList(qSeed, state.initialJointPosition);
        [qSol, ~] = solveTcpPoseIk(state.ikSolver, targetTform, seedList);
    catch MEik
        error('main_realtime_trajectory_fit_gui:TrajectoryIKFailed', ...
            '第 %d/%d 个轨迹点 IK 失败 [%s]: %s', ...
            i, numel(sampledPlan.tforms), sampledPlan.segmentNames{i}, MEik.message);
    end
    qSeries(i, :) = qSol;
    qSeed = qSol;
end
state.previewQSeries = qSeries;
state.previewTcpPath = rtfg_utils('computeTcpPath', state.robot, qSeries);
state.previewTargetZAxes = sampledPlan.zAxisPreview;
state.ui.cameraState = captureCurrentCameraState(state);
state = playPreviewJointSeries(state, qSeries);
state.currentQ = qSeries(end, :);
statusText = sprintf('尖端轨迹拟合完成。轨迹点数: %d', size(qSeries, 1));
end

function displayTraj = getDisplayTrajectory(state)
if isempty(state.traj)
    state.traj = generate_trajectory_3d(state.trajParams, state.envGeom);
end
displayTraj = rtfg_utils('applyPoseToTrajectory', state.traj, state.pose);
end

function state = playPreviewJointSeries(state, qSeries)
state.isAnimating = true;
try
    for k = 1:size(qSeries, 1)
        state.currentQ = qSeries(k, :);
        state = rtfg_render('renderScene', state);
        pause(0.02);
    end
catch ME
    state.isAnimating = false;
    rethrow(ME);
end
state.isAnimating = false;
end

function cameraState = captureCurrentCameraState(state)
cameraState = state.ui.cameraState;
if isfield(state.ui, 'mainAx') && isgraphics(state.ui.mainAx)
    cameraState = rtfg_render('captureCameraState', state.ui.mainAx, state.ui.cameraState);
end
end

function ikSolver = ensureIkSolver(robot)
ikSolver = inverseKinematics('RigidBodyTree', robot);
end

function [qSol, info] = solveTcpPoseIk(ikSolver, targetTform, qSeeds)
weightSchedule = {
    [1 1 1 0.20 0.20 0.20], deg2rad(30), "strict"
    [1 1 1 0.10 0.10 0.10], deg2rad(45), "relaxed"
    [1 1 1 0.03 0.03 0.03], deg2rad(70), "very_relaxed"
    [1 1 1 0.00 0.00 0.00], inf, "position_only"};

bestCandidate = struct('q', [], 'info', [], 'positionError', inf, 'orientationError', inf, 'mode', "");
robot = ikSolver.RigidBodyTree;

for i = 1:size(weightSchedule, 1)
    weights = weightSchedule{i, 1};
    orientationLimit = weightSchedule{i, 2};
    modeName = weightSchedule{i, 3};
    for j = 1:size(qSeeds, 1)
        qSeed = qSeeds(j, :);
        [qTry, infoTry] = ikSolver('sensor_shovel_tcp', targetTform, weights, qSeed);
        if any(~isfinite(qTry))
            continue;
        end
        [positionError, orientationError] = evaluateTcpPoseError(robot, qTry, targetTform);
        if positionError < bestCandidate.positionError
            bestCandidate = struct( ...
                'q', qTry, ...
                'info', infoTry, ...
                'positionError', positionError, ...
                'orientationError', orientationError, ...
                'mode', modeName);
        end
        if positionError <= 3e-2 && orientationError <= orientationLimit
            qSol = qTry;
            info = attachSolveMode(infoTry, modeName);
            return;
        end
    end
end

if isempty(bestCandidate.q)
    error('main_realtime_trajectory_fit_gui:IkReturnedInvalidValue', ...
        'IK 返回了非有限关节值');
end
if bestCandidate.positionError > 3e-2
    error('main_realtime_trajectory_fit_gui:IkPositionErrorTooLarge', ...
        '位置不可达，位置误差 %.6f m', bestCandidate.positionError);
end
error('main_realtime_trajectory_fit_gui:IkOrientationErrorTooLarge', ...
    '姿态误差过大 %.3f deg', rad2deg(bestCandidate.orientationError));
end

function [positionError, orientationError] = evaluateTcpPoseError(robot, q, targetTform)
actualTform = getTransform(robot, q, 'sensor_shovel_tcp');
positionError = norm(actualTform(1:3, 4) - targetTform(1:3, 4));
rotationDelta = targetTform(1:3, 1:3)' * actualTform(1:3, 1:3);
orientationError = acos(max(-1.0, min(1.0, (trace(rotationDelta) - 1) / 2)));
end

function infoOut = attachSolveMode(infoIn, modeName)
if isstruct(infoIn)
    infoOut = infoIn;
else
    infoOut = struct();
end
infoOut.SolveMode = char(modeName);
end

function qSeeds = buildIkSeedList(primarySeed, homeSeed)
qSeeds = [
    reshape(primarySeed, 1, [])
    reshape(homeSeed, 1, [])
    zeros(1, numel(primarySeed))];
qSeeds = unique(round(qSeeds, 12), 'rows', 'stable');
end

function targetPlan = buildTrajectoryTargetPlan(displayTraj)
trajIdx = trajectorySegmentIndices(displayTraj);
nPts = size(displayTraj.all, 1);
tforms = cell(nPts, 1);
segmentNames = strings(nPts, 1);
zAxisPreview = zeros(nPts, 6);

worldUp = [0 0 1];
seg2Start = trajIdx.seg2Start;
liftHeading = [];

for i = 1:nPts
    point = displayTraj.all(i, :);
    tangent = tangentAtTrajectoryPoint(displayTraj, i);
    if i < seg2Start
        if i <= trajIdx.seg0End
            segmentNames(i) = "斜线";
        else
            segmentNames(i) = "圆弧";
        end
        [R, zAxis] = buildArcPhaseRotation(tangent, worldUp);
        if i == trajIdx.seg2Start - 1
            liftHeading = horizontalHeadingFromTangent(tangent, worldUp);
        end
    else
        segmentNames(i) = "出泥";
        if isempty(liftHeading)
            backIdx = max(1, i - 1);
            liftHeading = horizontalHeadingFromTangent(tangentAtTrajectoryPoint(displayTraj, backIdx), worldUp);
        end
        [R, zAxis] = buildLiftPhaseRotation(liftHeading, worldUp);
    end
    tforms{i} = rtfg_utils('rt2tform', R, point);
    zAxisPreview(i, :) = [point, zAxis];
end

targetPlan = struct();
targetPlan.points = displayTraj.all;
targetPlan.tforms = tforms;
targetPlan.segmentNames = cellstr(segmentNames);
targetPlan.segmentTransitionIndices = [trajIdx.seg1Start; trajIdx.seg2Start];
targetPlan.zAxisPreview = zAxisPreview;
targetPlan.segmentIndices = trajIdx;
end

function trajIdx = trajectorySegmentIndices(traj)
n0 = size(traj.seg0, 1);
n1 = size(traj.seg1, 1);
n2 = size(traj.seg2, 1);

seg0End = max(1, n0 - 1);
seg1Start = seg0End + 1;
seg1End = seg0End + n1;
seg2Start = seg1End + 1;
seg2End = seg1End + max(0, n2 - 1);

trajIdx = struct( ...
    'seg0Start', 1, ...
    'seg0End', seg0End, ...
    'seg1Start', seg1Start, ...
    'seg1End', seg1End, ...
    'seg2Start', seg2Start, ...
    'seg2End', seg2End);
end

function tangent = tangentAtTrajectoryPoint(traj, idx)
if isfield(traj, 'tangent3d') && ~isempty(traj.tangent3d)
    tangent = traj.tangent3d(idx, :);
else
    nPts = size(traj.all, 1);
    if idx == 1
        tangent = traj.all(2, :) - traj.all(1, :);
    elseif idx == nPts
        tangent = traj.all(end, :) - traj.all(end - 1, :);
    else
        tangent = traj.all(idx + 1, :) - traj.all(idx - 1, :);
    end
end
tangent = rtfg_utils('normalizeRowVector', tangent, [0 1 0]);
end

function [R, zAxis] = buildArcPhaseRotation(tangent, worldUp)
negXPath = rtfg_utils('normalizeRowVector', tangent, [1 0 0]);
xAxis = -negXPath;
zProj = worldUp - dot(worldUp, xAxis) * xAxis;
zAxis = rtfg_utils('normalizeRowVector', zProj, worldUp);
if zAxis(3) < 0
    zAxis = -zAxis;
end
yAxis = cross(zAxis, xAxis);
yAxis = rtfg_utils('normalizeRowVector', yAxis, [0 1 0]);
zAxis = cross(xAxis, yAxis);
zAxis = rtfg_utils('normalizeRowVector', zAxis, worldUp);
R = [xAxis(:), yAxis(:), zAxis(:)];
end

function heading = horizontalHeadingFromTangent(tangent, worldUp)
heading = tangent - dot(tangent, worldUp) * worldUp;
heading = rtfg_utils('normalizeRowVector', heading, [0 1 0]);
end

function [R, zAxis] = buildLiftPhaseRotation(horizontalHeading, worldUp)
zAxis = rtfg_utils('normalizeRowVector', worldUp, [0 0 1]);
negXPath = rtfg_utils('normalizeRowVector', horizontalHeading, [1 0 0]);
xAxis = -negXPath;
yAxis = cross(zAxis, xAxis);
yAxis = rtfg_utils('normalizeRowVector', yAxis, [0 1 0]);
xAxis = cross(yAxis, zAxis);
xAxis = rtfg_utils('normalizeRowVector', xAxis, [-1 0 0]);
R = [xAxis(:), yAxis(:), zAxis(:)];
end
