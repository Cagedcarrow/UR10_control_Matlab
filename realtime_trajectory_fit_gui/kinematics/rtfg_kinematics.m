function varargout = rtfg_kinematics(action, varargin)
switch action
    case 'moveToTrajectoryStart'
        [varargout{1}, varargout{2}] = moveToTrajectoryStart(varargin{1});
    case 'trackTrajectory'
        [varargout{1}, varargout{2}] = trackTrajectory(varargin{1});
    case 'playPreviewTrajectory'
        [varargout{1}, varargout{2}] = playPreviewTrajectory(varargin{1});
    otherwise
        error('rtfg_kinematics:UnknownAction', 'Unknown action: %s', action);
end
end

function [state, statusText] = moveToTrajectoryStart(state)
targetPlan = buildTrajectoryTargetPlan(getDisplayTrajectory(state));
targetTform = targetPlan.tforms{1};
startPlan = sampleTargetPlanByIndices(targetPlan, 1);
state.ikSolver = ensureIkSolver(state.robot);
seedList = buildIkSeedList(state.robot, state.currentQ, state.initialJointPosition);
state = updateProgressStatus(state, '正在求解移动到起始点...', 0.20);
[qStart, info] = solveTcpPoseIk(state, targetTform, seedList, state.currentQ, zeros(1, numel(state.currentQ)));
qSeries = rtfg_quintic_joint_series(state.currentQ, qStart, 70);
state.previewAnchorQSeries = qStart;
state.previewQSeries = qSeries;
state.ui.cameraState = captureCurrentCameraState(state);
state = updateProgressStatus(state, sprintf('正在播放移动到起始点 0/%d ...', size(qSeries, 1)), 0.55);
state = playPreviewJointSeries(state, qSeries);
state.previewTcpPath = rtfg_utils('computeTcpPath', state.robot, qSeries);
state.previewTargetZAxes = targetPlan.zAxisPreview;
moveSegmentNames = repmat({'移动到起始点'}, size(qSeries, 1), 1);
state.collisionResults = rtfg_collision('evaluateTrajectory', state, qSeries, moveSegmentNames);
state.previewMetrics = computePreviewMetrics(state.robot, startPlan, startPlan, qStart, qSeries, state.currentQ);
state.currentQ = qStart;
statusText = composeKinematicsStatus( ...
    rtfg_collision('summarizeResults', state.collisionResults, ...
    sprintf('已到达轨迹起始点。IK 退出标志: %s', rtfg_utils('stringifyIkStatus', info))), ...
    state.previewMetrics);
end

function [state, statusText] = trackTrajectory(state)
targetPlan = buildTrajectoryTargetPlan(getDisplayTrajectory(state));
try
    state = updateProgressStatus(state, sprintf('正在准备 MEX 轨迹拟合输入 0/%d ...', ...
        numel(targetPlan.tforms)), 0.02);
    solverResult = runMexTrajectorySolver(state, targetPlan);
    anchorQSeries = solverResult.anchorQSeries;
    playbackQSeries = solverResult.playbackQSeries;
    state.previewAnchorQSeries = anchorQSeries;
    state.previewQSeries = playbackQSeries;
    state.previewTcpPath = solverResult.tcpPath;
    state.previewTargetZAxes = targetPlan.zAxisPreview;
    state.previewMetrics = solverResult.previewMetrics;
    state.collisionResults = solverResult.collisionResults;
    statusText = composeKinematicsStatus( ...
        rtfg_collision('summarizeResults', state.collisionResults, ...
        sprintf('MEX 尖端轨迹拟合完成。锚点数: %d，回放点数: %d。已解锁开始运行按钮', size(anchorQSeries, 1), size(playbackQSeries, 1))), ...
        state.previewMetrics);
catch ME
    [state, statusText] = trackTrajectoryLegacy(state, targetPlan, ME);
end
end

function [state, statusText] = trackTrajectoryLegacy(state, targetPlan, causeME)
state.ikSolver = ensureIkSolver(state.robot);
anchorPlan = buildAdaptiveAnchorPlan(targetPlan);
state = updateProgressStatus(state, sprintf('MEX 失败，正在回退 MATLAB 主链 0/%d ...', ...
    numel(anchorPlan.tforms)), 0.05);
[anchorQSeries, state] = solveAnchorTrajectory(state, anchorPlan);
[playbackQSeries, playbackSegmentNames, state] = buildPlaybackJointSeries(state, anchorQSeries, anchorPlan, targetPlan);
state = updateProgressStatus(state, sprintf('正在进行回放轨迹碰撞复检 0/%d ...', size(playbackQSeries, 1)), 0.62);
collisionResults = rtfg_collision('evaluateTrajectory', state, playbackQSeries, playbackSegmentNames);
state.previewAnchorQSeries = anchorQSeries;
state.previewQSeries = playbackQSeries;
state.previewTcpPath = rtfg_utils('computeTcpPath', state.robot, playbackQSeries);
state.previewTargetZAxes = anchorPlan.zAxisPreview;
state.previewMetrics = computePreviewMetrics(state.robot, targetPlan, anchorPlan, anchorQSeries, playbackQSeries, state.currentQ);
state.collisionResults = collisionResults;
statusText = composeKinematicsStatus( ...
    rtfg_collision('summarizeResults', state.collisionResults, ...
    sprintf('MEX 求解失败，已回退 MATLAB 主链。失败原因: %s。锚点数: %d，回放点数: %d。已解锁开始运行按钮', ...
    causeME.message, size(anchorQSeries, 1), size(playbackQSeries, 1))), ...
    state.previewMetrics);
end

function solverResult = runMexTrajectorySolver(state, targetPlan)
if exist('rtfg_solver_mex', 'file') ~= 3
    error('main_realtime_trajectory_fit_gui:MexSolverUnavailable', ...
        ['未找到已编译的 rtfg_solver_mex。请先在 MATLAB 中执行：' newline ...
         'addpath(fullfile(pwd, ''realtime_trajectory_fit_gui'', ''mex''));' newline ...
         'build_mex']);
end

tformArray = zeros(4, 4, numel(targetPlan.tforms));
if isempty(state.collisionEnv) || ~isfield(state.collisionEnv, 'robotBasePose') || isempty(state.collisionEnv.robotBasePose)
    error('main_realtime_trajectory_fit_gui:MexMissingCollisionEnv', ...
        'MEX 求解前未初始化 collisionEnv.robotBasePose');
end
TrobotToBase = invertRigidTformLocal(state.collisionEnv.robotBasePose);
for i = 1:numel(targetPlan.tforms)
    tformArray(:, :, i) = TrobotToBase * targetPlan.tforms{i};
end

inputStruct = struct();
inputStruct.sceneUrdf = state.paths.collisionRobotUrdf;
inputStruct.collisionUrdf = state.paths.collisionRobotUrdf;
inputStruct.baseLink = 'ur10';
inputStruct.tipLink = 'sensor_shovel_tcp';
qWarmStart = computeMexWarmStart(state, targetPlan);
inputStruct.currentQ = reshape(qWarmStart, 1, []);
inputStruct.initialQ = reshape(qWarmStart, 1, []);
inputStruct.targetTforms = tformArray;
inputStruct.segmentNames = targetPlan.segmentNames(:);
inputStruct.basinBoxes = buildMexBasinBoxes(state);
inputStruct.clearanceThreshold = state.collisionConfig.clearanceThreshold;

solverResult = rtfg_solver_mex(inputStruct);
if ~isstruct(solverResult) || ~isfield(solverResult, 'anchorQSeries') || ~isfield(solverResult, 'playbackQSeries')
    error('main_realtime_trajectory_fit_gui:MexSolverBadOutput', ...
        'rtfg_solver_mex 返回结果结构不完整');
end

function qWarmStart = computeMexWarmStart(state, targetPlan)
state.ikSolver = ensureIkSolver(state.robot);
seedList = buildIkSeedList(state.robot, state.currentQ, state.initialJointPosition);
[qWarmStart, ~] = solveTcpPoseIk(state, targetPlan.tforms{1}, seedList, state.currentQ, zeros(1, numel(state.currentQ)));
end
end

function basinBoxes = buildMexBasinBoxes(state)
if isempty(state.collisionEnv) || ~isfield(state.collisionEnv, 'basinBoxes') || isempty(state.collisionEnv.basinBoxes)
    error('main_realtime_trajectory_fit_gui:MexMissingBasinBoxes', ...
        'MEX 求解前未初始化 collisionEnv.basinBoxes');
end
boxSpecs = state.collisionEnv.basinBoxes;
basinBoxes = repmat(struct('name', '', 'size', [], 'pose', []), numel(boxSpecs), 1);
for i = 1:numel(boxSpecs)
    basinBoxes(i).name = boxSpecs(i).name;
    basinBoxes(i).size = boxSpecs(i).size;
    basinBoxes(i).pose = boxSpecs(i).poseRobot;
end
end

function Tinv = invertRigidTformLocal(T)
R = T(1:3, 1:3);
p = T(1:3, 4);
Tinv = eye(4);
Tinv(1:3, 1:3) = R.';
Tinv(1:3, 4) = -R.' * p;
end

function [state, statusText] = playPreviewTrajectory(state)
if isempty(state.previewQSeries)
    error('main_realtime_trajectory_fit_gui:NoPreviewTrajectory', '尚未生成可播放的拟合轨迹');
end
playSeries = state.previewQSeries;
if norm(wrapJointDelta(playSeries(1, :) - state.currentQ)) > deg2rad(0.25)
    moveSeries = rtfg_quintic_joint_series(state.currentQ, playSeries(1, :), 40);
    playSeries = [moveSeries(1:end-1, :); playSeries];
end
state.ui.cameraState = captureCurrentCameraState(state);
state = updateProgressStatus(state, sprintf('正在播放连续回放轨迹 0/%d ...', size(playSeries, 1)), 0.75);
state = playPreviewJointSeries(state, playSeries);
state.currentQ = playSeries(end, :);
statusText = composeKinematicsStatus('拟合轨迹播放完成。', state.previewMetrics);
end

function displayTraj = getDisplayTrajectory(state)
if isempty(state.traj)
    state.traj = generate_trajectory_3d(state.trajParams, state.envGeom);
end
displayTraj = rtfg_utils('applyPoseToTrajectory', state.traj, state.pose);
end

function state = playPreviewJointSeries(state, qSeries)
state.isAnimating = true;
renderStride = choosePlaybackRenderStride(size(qSeries, 1));
try
    for k = 1:size(qSeries, 1)
        state.currentQ = qSeries(k, :);
        if k == 1 || k == size(qSeries, 1) || mod(k, 25) == 0
            frac = 0.75 + 0.25 * (k / max(size(qSeries, 1), 1));
            state = updateProgressStatus(state, sprintf('正在播放连续回放轨迹 %d/%d ...', k, size(qSeries, 1)), frac);
        end
        if k == 1 || k == size(qSeries, 1) || mod(k - 1, renderStride) == 0
            state = rtfg_render('renderScene', state);
        end
        pause(0.001);
    end
catch ME
    state.isAnimating = false;
    rethrow(ME);
end
state.isAnimating = false;
end

function cameraState = captureCurrentCameraState(state)
cameraState = state.ui.cameraState;
if isfield(state.ui, 'mainAx') && ~isempty(state.ui.mainAx) && isgraphics(state.ui.mainAx)
    cameraState = rtfg_render('captureCameraState', state.ui.mainAx, state.ui.cameraState);
end
end

function state = updateProgressStatus(state, text, progress)
if isfield(state, 'ui') && isfield(state.ui, 'lblStatus') && ~isempty(state.ui.lblStatus) && isgraphics(state.ui.lblStatus)
    state = rtfg_ui('setStatus', state, text);
end
if nargin >= 3 && isfield(state, 'ui') && isfield(state.ui, 'gaugeProgress') && ~isempty(state.ui.gaugeProgress) && isgraphics(state.ui.gaugeProgress)
    state = rtfg_ui('setProgress', state, progress, text);
end
end

function ikSolver = ensureIkSolver(robot)
ikSolver = inverseKinematics('RigidBodyTree', robot);
end

function [qSol, info] = solveTcpPoseIk(state, targetTform, qSeeds, qPrev, dqPrev)
ikSolver = state.ikSolver;
weightSchedule = {
    [1 1 1 0.20 0.20 0.20], deg2rad(30), "strict"
    [1 1 1 0.10 0.10 0.10], deg2rad(45), "relaxed"
    [1 1 1 0.03 0.03 0.03], deg2rad(70), "very_relaxed"
    [1 1 1 0.00 0.00 0.00], inf, "position_only"};

bestSafeCandidate = struct('q', [], 'info', [], 'positionError', inf, ...
    'orientationError', inf, 'mode', "", 'cost', inf, 'clearance', -inf, 'failureReason', '');
bestFallback = struct('q', [], 'info', [], 'positionError', inf, ...
    'orientationError', inf, 'mode', "", 'cost', inf, 'clearance', -inf, 'failureReason', '');
robot = ikSolver.RigidBodyTree;
if nargin < 5 || isempty(dqPrev)
    dqPrev = zeros(1, numel(qPrev));
end

seedStages = {
    qSeeds
    expandIkSeedList(robot, qSeeds)};

for stageIdx = 1:numel(seedStages)
    stageSeeds = seedStages{stageIdx};
    for i = 1:size(weightSchedule, 1)
        weights = weightSchedule{i, 1};
        orientationLimit = weightSchedule{i, 2};
        modeName = weightSchedule{i, 3};
        for j = 1:size(stageSeeds, 1)
            qSeed = stageSeeds(j, :);
            [qTry, infoTry] = ikSolver('sensor_shovel_tcp', targetTform, weights, qSeed);
            if any(~isfinite(qTry))
                continue;
            end
            qTry = alignEquivalentConfiguration(robot, qTry, qPrev, dqPrev);
            [positionError, orientationError] = evaluateTcpPoseError(robot, qTry, targetTform);
            collisionMetrics = rtfg_collision('evaluateConfiguration', state, qTry);
            minClearance = min([collisionMetrics.minSelfClearance, ...
                collisionMetrics.minToolBodyClearance, collisionMetrics.minToolBasinClearance]);
            candidate = struct( ...
                'q', qTry, ...
                'info', infoTry, ...
                'positionError', positionError, ...
                'orientationError', orientationError, ...
                'mode', modeName, ...
                'cost', continuityCost(qTry, qPrev, dqPrev), ...
                'clearance', minClearance, ...
                'failureReason', describeClearanceFailure(collisionMetrics));
            bestFallback = updateBestFallback(bestFallback, candidate);
            if positionError <= 3e-2 && orientationError <= orientationLimit && ...
                    minClearance >= state.collisionConfig.clearanceThreshold
                bestSafeCandidate = updateBestSafe(bestSafeCandidate, candidate);
            end
        end
        if ~isempty(bestSafeCandidate.q) && strcmp(bestSafeCandidate.mode, modeName)
            qSol = bestSafeCandidate.q;
            info = attachSolveMode(bestSafeCandidate.info, modeName, bestSafeCandidate.clearance);
            return;
        end
    end
end

if isempty(bestFallback.q)
    error('main_realtime_trajectory_fit_gui:IkReturnedInvalidValue', ...
        'IK 返回了非有限关节值');
end
if bestFallback.positionError > 3e-2
    error('main_realtime_trajectory_fit_gui:IkPositionErrorTooLarge', ...
        '位置不可达，位置误差 %.6f m', bestFallback.positionError);
end
if bestFallback.clearance < state.collisionConfig.clearanceThreshold
    error('main_realtime_trajectory_fit_gui:IkClearanceTooSmall', ...
        'clearance 仅 %.3f mm: %s', 1000 * bestFallback.clearance, bestFallback.failureReason);
end
error('main_realtime_trajectory_fit_gui:IkOrientationErrorTooLarge', ...
    '姿态误差过大 %.3f deg', rad2deg(bestFallback.orientationError));
end

function [positionError, orientationError] = evaluateTcpPoseError(robot, q, targetTform)
actualTform = getTransform(robot, q, 'sensor_shovel_tcp');
positionError = norm(actualTform(1:3, 4) - targetTform(1:3, 4));
rotationDelta = targetTform(1:3, 1:3)' * actualTform(1:3, 1:3);
orientationError = acos(max(-1.0, min(1.0, (trace(rotationDelta) - 1) / 2)));
end

function infoOut = attachSolveMode(infoIn, modeName, clearance)
if isstruct(infoIn)
    infoOut = infoIn;
else
    infoOut = struct();
end
infoOut.SolveMode = char(modeName);
if nargin >= 3
    infoOut.MinClearance = clearance;
end
end

function qSeeds = buildIkSeedList(robot, primarySeed, homeSeed)
qSeeds = [
    reshape(primarySeed, 1, [])
    reshape(homeSeed, 1, [])
    zeros(1, numel(primarySeed))];
qSeeds = sanitizeSeedList(robot, qSeeds);
end

function qSeeds = expandIkSeedList(robot, baseSeeds)
expanded = baseSeeds;
wraps = [-2*pi, 0, 2*pi];
for i = 1:size(baseSeeds, 1)
    q0 = baseSeeds(i, :);
    for w5 = wraps
        for w6 = wraps
            if w5 == 0 && w6 == 0
                continue;
            end
            qAlt = q0;
            if numel(qAlt) >= 5
                qAlt(5) = qAlt(5) + w5;
            end
            if numel(qAlt) >= 6
                qAlt(6) = qAlt(6) + w6;
            end
            expanded(end + 1, :) = qAlt; %#ok<AGROW>
        end
    end
end
qSeeds = sanitizeSeedList(robot, expanded);
end

function qSeeds = sanitizeSeedList(robot, qSeedsIn)
jointLimits = movableJointLimits(robot);
qSeeds = zeros(0, size(qSeedsIn, 2));
for i = 1:size(qSeedsIn, 1)
    q = qSeedsIn(i, :);
    for j = 1:numel(q)
        q(j) = atan2(sin(q(j)), cos(q(j)));
        lower = jointLimits(j, 1);
        upper = jointLimits(j, 2);
        if isfinite(lower) && q(j) < lower
            q(j) = lower;
        end
        if isfinite(upper) && q(j) > upper
            q(j) = upper;
        end
    end
    qSeeds(end + 1, :) = q; %#ok<AGROW>
end
qSeeds = unique(round(qSeeds, 12), 'rows', 'stable');
end

function limits = movableJointLimits(robot)
limits = zeros(numel(robot.homeConfiguration), 2);
idx = 0;
for i = 1:numel(robot.Bodies)
    joint = robot.Bodies{i}.Joint;
    if strcmp(joint.Type, 'fixed')
        continue;
    end
    idx = idx + 1;
    limits(idx, :) = joint.PositionLimits;
end
end

function best = updateBestFallback(best, candidate)
if isempty(best.q)
    best = candidate;
    return;
end
if candidate.positionError < best.positionError - 1e-12
    best = candidate;
    return;
end
if abs(candidate.positionError - best.positionError) <= 1e-12
    if candidate.clearance > best.clearance + 1e-12
        best = candidate;
        return;
    end
    if abs(candidate.clearance - best.clearance) <= 1e-12 && candidate.cost < best.cost
        best = candidate;
    end
end
end

function best = updateBestSafe(best, candidate)
if isempty(best.q)
    best = candidate;
    return;
end
if candidate.cost < best.cost - 1e-12
    best = candidate;
    return;
end
if abs(candidate.cost - best.cost) <= 1e-12 && candidate.clearance > best.clearance + 1e-12
    best = candidate;
end
end

function reason = describeClearanceFailure(metrics)
minValues = [metrics.minToolBasinClearance, metrics.minToolBodyClearance, metrics.minSelfClearance];
names = {metrics.minToolBasinObject, metrics.minToolBodyObject, metrics.minSelfObject};
[minValue, idx] = min(minValues);
if ~isfinite(minValue)
    reason = '无有效 clearance 数据';
else
    reason = sprintf('%s', names{idx});
end
end

function targetPlan = buildTrajectoryTargetPlan(displayTraj)
rawIdx = trajectorySegmentIndices(displayTraj);
nPts = size(displayTraj.all, 1);

worldUp = [0 0 1];
allTrackRot = zeros(3, 3, nPts);
for i = 1:nPts
    tangent = tangentAtTrajectoryPoint(displayTraj, i);
    [allTrackRot(:, :, i), ~] = buildArcPhaseRotation(tangent, worldUp);
end

liftHeading = averageHorizontalHeading(displayTraj, rawIdx.seg1Start:rawIdx.seg1End, worldUp);
[liftRotation, ~] = buildLiftPhaseRotation(liftHeading, worldUp);
sharedLiftPoint = displayTraj.all(rawIdx.seg1End, :);
sharedStartRotation = allTrackRot(:, :, rawIdx.seg1End);
sharedTransitionRotations = buildSharedLiftTransitionRotations(sharedStartRotation, liftRotation);

points = zeros(0, 3);
tforms = {};
segmentNames = strings(0, 1);
zAxisPreview = zeros(0, 6);

[points, tforms, segmentNames, zAxisPreview] = appendPlanRange( ...
    points, tforms, segmentNames, zAxisPreview, ...
    displayTraj.all(1:rawIdx.seg0End, :), allTrackRot(:, :, 1:rawIdx.seg0End), "斜线");
[points, tforms, segmentNames, zAxisPreview] = appendPlanRange( ...
    points, tforms, segmentNames, zAxisPreview, ...
    displayTraj.all(rawIdx.seg1Start:rawIdx.seg1End - 1, :), allTrackRot(:, :, rawIdx.seg1Start:rawIdx.seg1End - 1), "圆弧");
[points, tforms, segmentNames, zAxisPreview] = appendPlanRange( ...
    points, tforms, segmentNames, zAxisPreview, ...
    repmat(sharedLiftPoint, size(sharedTransitionRotations, 3), 1), sharedTransitionRotations, "出泥");
[points, tforms, segmentNames, zAxisPreview] = appendPlanRange( ...
    points, tforms, segmentNames, zAxisPreview, ...
    displayTraj.all(rawIdx.seg2Start:rawIdx.seg2End, :), repmat(liftRotation, 1, 1, rawIdx.seg2End - rawIdx.seg2Start + 1), "出泥");

trajIdx = struct( ...
    'seg0Start', 1, ...
    'seg0End', rawIdx.seg0End, ...
    'seg1Start', rawIdx.seg1Start, ...
    'seg1End', rawIdx.seg1End - 1, ...
    'seg2Start', rawIdx.seg1End, ...
    'seg2End', size(points, 1));

targetPlan = struct();
targetPlan.points = points;
targetPlan.tforms = tforms;
targetPlan.segmentNames = cellstr(segmentNames);
targetPlan.segmentTransitionIndices = [trajIdx.seg1Start; trajIdx.seg1End; trajIdx.seg2Start];
targetPlan.transitionWindowIndices = trajIdx.seg2Start;
targetPlan.zAxisPreview = zAxisPreview;
targetPlan.segmentIndices = trajIdx;
end

function [points, tforms, segmentNames, zAxisPreview] = appendPlanRange(points, tforms, segmentNames, zAxisPreview, newPoints, newRotations, segmentName)
for i = 1:size(newPoints, 1)
    [points, tforms, segmentNames, zAxisPreview] = appendPlanPose( ...
        points, tforms, segmentNames, zAxisPreview, newPoints(i, :), newRotations(:, :, i), segmentName);
end
end

function [points, tforms, segmentNames, zAxisPreview] = appendPlanPose(points, tforms, segmentNames, zAxisPreview, point, rotation, segmentName)
points(end + 1, :) = point; %#ok<AGROW>
tforms{end + 1, 1} = rtfg_utils('rt2tform', rotation, point); %#ok<AGROW>
segmentNames(end + 1, 1) = string(segmentName); %#ok<AGROW>
zAxisPreview(end + 1, :) = [point, rotation(:, 3).']; %#ok<AGROW>
end

function rotations = buildSharedLiftTransitionRotations(R0, R1)
rotDelta = rotationDistance(R0, R1);
nSteps = min(max(8, ceil(rotDelta / deg2rad(3.0)) + 1), 18);
rotations = zeros(3, 3, nSteps);
for i = 1:nSteps
    alpha = transitionBlendAlpha(i, 0, nSteps);
    rotations(:, :, i) = slerpRotation(R0, R1, alpha);
end
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

function transitionWindow = buildLiftTransitionWindow(traj, trajIdx)
arcLen = polylineLength(traj.seg1);
liftLen = polylineLength(traj.seg2);
preLength = min(max(0.020, 0.22 * arcLen), 0.080);
postLength = min(max(0.015, 0.18 * liftLen), 0.060);

cumulative = [0; cumsum(vecnorm(diff(traj.all, 1, 1), 2, 2))];
transitionS = cumulative(trajIdx.seg2Start);
startIdx = find(cumulative >= transitionS - preLength, 1, 'first');
endIdx = find(cumulative >= transitionS + postLength, 1, 'first');

if isempty(startIdx)
    startIdx = max(trajIdx.seg1Start, trajIdx.seg2Start - 1);
end
if isempty(endIdx)
    endIdx = min(size(traj.all, 1), trajIdx.seg2Start + 1);
end

startIdx = max(trajIdx.seg1Start, min(startIdx, trajIdx.seg2Start - 1));
endIdx = min(trajIdx.seg2End, max(endIdx, trajIdx.seg2Start + 1));
headingRefStart = max(trajIdx.seg1Start, startIdx);
headingRefEnd = max(headingRefStart, trajIdx.seg2Start - 1);

transitionWindow = struct( ...
    'startIdx', startIdx, ...
    'endIdx', endIdx, ...
    'headingRefIndices', headingRefStart:headingRefEnd);
end

function heading = averageHorizontalHeading(traj, indices, worldUp)
headingVec = zeros(1, 3);
for i = indices
    tangent = tangentAtTrajectoryPoint(traj, i);
    proj = tangent - dot(tangent, worldUp) * worldUp;
    if norm(proj) > 1e-9
        headingVec = headingVec + proj;
    end
end
if norm(headingVec) < 1e-9
    for i = fliplr(indices)
        tangent = tangentAtTrajectoryPoint(traj, i);
        proj = tangent - dot(tangent, worldUp) * worldUp;
        if norm(proj) > 1e-9
            headingVec = proj;
            break;
        end
    end
end
heading = rtfg_utils('normalizeRowVector', headingVec, [0 1 0]);
end

function alpha = transitionBlendAlpha(idx, startIdx, endIdx)
if endIdx <= startIdx
    alpha = 1.0;
    return;
end
t = (idx - startIdx) / max(endIdx - startIdx, 1);
t = min(max(t, 0.0), 1.0);
alpha = 10 * t^3 - 15 * t^4 + 6 * t^5;
end

function R = slerpRotation(R0, R1, alpha)
q0 = rotmToQuatWxyz(R0);
q1 = rotmToQuatWxyz(R1);
if dot(q0, q1) < 0
    q1 = -q1;
end
cosTheta = min(max(dot(q0, q1), -1.0), 1.0);
if cosTheta > 1 - 1e-10
    q = (1 - alpha) * q0 + alpha * q1;
    q = q / norm(q);
else
    theta = acos(cosTheta);
    sinTheta = sin(theta);
    w0 = sin((1 - alpha) * theta) / sinTheta;
    w1 = sin(alpha * theta) / sinTheta;
    q = w0 * q0 + w1 * q1;
end
R = quatWxyzToRotm(q / norm(q));
end

function q = rotmToQuatWxyz(R)
traceR = trace(R);
if traceR > 0
    s = 2 * sqrt(traceR + 1.0);
    qw = 0.25 * s;
    qx = (R(3, 2) - R(2, 3)) / s;
    qy = (R(1, 3) - R(3, 1)) / s;
    qz = (R(2, 1) - R(1, 2)) / s;
elseif R(1, 1) > R(2, 2) && R(1, 1) > R(3, 3)
    s = 2 * sqrt(1.0 + R(1, 1) - R(2, 2) - R(3, 3));
    qw = (R(3, 2) - R(2, 3)) / s;
    qx = 0.25 * s;
    qy = (R(1, 2) + R(2, 1)) / s;
    qz = (R(1, 3) + R(3, 1)) / s;
elseif R(2, 2) > R(3, 3)
    s = 2 * sqrt(1.0 + R(2, 2) - R(1, 1) - R(3, 3));
    qw = (R(1, 3) - R(3, 1)) / s;
    qx = (R(1, 2) + R(2, 1)) / s;
    qy = 0.25 * s;
    qz = (R(2, 3) + R(3, 2)) / s;
else
    s = 2 * sqrt(1.0 + R(3, 3) - R(1, 1) - R(2, 2));
    qw = (R(2, 1) - R(1, 2)) / s;
    qx = (R(1, 3) + R(3, 1)) / s;
    qy = (R(2, 3) + R(3, 2)) / s;
    qz = 0.25 * s;
end
q = [qw, qx, qy, qz];
q = q / norm(q);
end

function R = quatWxyzToRotm(q)
q = q / norm(q);
qw = q(1); qx = q(2); qy = q(3); qz = q(4);
R = [ ...
    1 - 2 * (qy^2 + qz^2), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw); ...
    2 * (qx * qy + qz * qw), 1 - 2 * (qx^2 + qz^2), 2 * (qy * qz - qx * qw); ...
    2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx^2 + qy^2)];
end

function lengthOut = polylineLength(pts)
if size(pts, 1) < 2
    lengthOut = 0.0;
    return;
end
lengthOut = sum(vecnorm(diff(pts, 1, 1), 2, 2));
end

function [anchorQSeries, state] = solveAnchorTrajectory(state, anchorPlan)
nAnchors = numel(anchorPlan.tforms);
anchorQSeries = zeros(nAnchors, numel(state.currentQ));
qPrev = state.currentQ;
dqPrev = zeros(1, numel(state.currentQ));
for i = 1:nAnchors
    targetTform = anchorPlan.tforms{i};
    try
        seedList = buildIkSeedList(state.robot, qPrev, state.initialJointPosition);
        [qSol, ~] = solveTcpPoseIk(state, targetTform, seedList, qPrev, dqPrev);
    catch MEik
        error('main_realtime_trajectory_fit_gui:TrajectoryIKFailed', ...
            '第 %d/%d 个锚点 IK 失败 [%s]: %s', ...
            i, nAnchors, anchorPlan.segmentNames{i}, MEik.message);
    end
    anchorQSeries(i, :) = qSol;
    dqPrev = wrapJointDelta(qSol - qPrev);
    qPrev = qSol;
    if i == 1 || i == nAnchors || mod(i, 10) == 0
        state = updateProgressStatus(state, sprintf('正在拟合轨迹锚点 %d/%d ...', i, nAnchors));
    end
end
end

function anchorPlan = buildAdaptiveAnchorPlan(targetPlan)
nPts = size(targetPlan.points, 1);
mustKeep = unique([1; targetPlan.segmentTransitionIndices(:); targetPlan.transitionWindowIndices(:); nPts]);
if nPts <= numel(mustKeep)
    anchorPlan = sampleTargetPlanByIndices(targetPlan, mustKeep);
    return;
end

idx = unique([mustKeep; round(linspace(1, nPts, min(nPts, 72))).']);
idx = sort(idx);
maxAnchors = min(max(120, ceil(numel(idx) * 2.4)), nPts);
changed = true;
while changed
    changed = false;
    inserts = zeros(0, 1);
    for k = 1:numel(idx) - 1
        i0 = idx(k);
        i1 = idx(k + 1);
        if i1 <= i0 + 1
            continue;
        end
        if shouldSplitTargetInterval(targetPlan, i0, i1)
            mid = round((i0 + i1) / 2);
            if all(idx ~= mid)
                inserts(end + 1, 1) = mid; %#ok<AGROW>
            end
        end
    end
    if ~isempty(inserts)
        changed = true;
        idx = unique([idx; inserts]);
        idx = sort(idx);
        if numel(idx) >= maxAnchors
            break;
        end
    end
end

if numel(idx) > maxAnchors
    idx = reduceAnchorIndices(targetPlan, idx, mustKeep, maxAnchors);
end
anchorPlan = sampleTargetPlanByIndices(targetPlan, idx);
end

function tf = shouldSplitTargetInterval(targetPlan, idx0, idx1)
pts = targetPlan.points(idx0:idx1, :);
pathLength = polylineLength(pts);
R0 = targetPlan.tforms{idx0}(1:3, 1:3);
R1 = targetPlan.tforms{idx1}(1:3, 1:3);
rotDelta = rotationDistance(R0, R1);
tangent0 = -R0(:, 1).';
tangent1 = -R1(:, 1).';
tangentDelta = acos(max(-1.0, min(1.0, dot(tangent0, tangent1))));
tf = pathLength > 0.012 || rotDelta > deg2rad(0.55) || tangentDelta > deg2rad(0.50);
end

function idx = reduceAnchorIndices(targetPlan, idxIn, mustKeep, maxAnchors)
idx = sort(idxIn(:));
while numel(idx) > maxAnchors
    optional = setdiff(idx, mustKeep, 'stable');
    if isempty(optional)
        break;
    end
    bestRemoval = optional(1);
    bestCost = inf;
    for i = 1:numel(optional)
        candidate = optional(i);
        pos = find(idx == candidate, 1, 'first');
        if isempty(pos) || pos == 1 || pos == numel(idx)
            continue;
        end
        left = idx(pos - 1);
        right = idx(pos + 1);
        mergeCost = intervalComplexity(targetPlan, left, right);
        if mergeCost < bestCost
            bestCost = mergeCost;
            bestRemoval = candidate;
        end
    end
    idx(idx == bestRemoval) = [];
end
end

function cost = intervalComplexity(targetPlan, idx0, idx1)
pts = targetPlan.points(idx0:idx1, :);
pathLength = polylineLength(pts);
R0 = targetPlan.tforms{idx0}(1:3, 1:3);
R1 = targetPlan.tforms{idx1}(1:3, 1:3);
rotDelta = rotationDistance(R0, R1);
tangent0 = -R0(:, 1).';
tangent1 = -R1(:, 1).';
tangentDelta = acos(max(-1.0, min(1.0, dot(tangent0, tangent1))));
cost = pathLength + 0.5 * rotDelta + 0.25 * tangentDelta;
end

function sampledPlan = sampleTargetPlanByIndices(targetPlan, idx)
idx = unique(idx(:));
sampledPlan = targetPlan;
sampledPlan.points = targetPlan.points(idx, :);
sampledPlan.tforms = targetPlan.tforms(idx);
sampledPlan.segmentNames = targetPlan.segmentNames(idx);
sampledPlan.zAxisPreview = targetPlan.zAxisPreview(idx, :);
sampledPlan.sampleIndices = idx;
end

function [playbackQSeries, playbackSegmentNames, state] = buildPlaybackJointSeries(state, anchorQSeries, anchorPlan, targetPlan)
if isempty(anchorQSeries)
    playbackQSeries = zeros(0, numel(state.currentQ));
    playbackSegmentNames = cell(0, 1);
    return;
end

playbackQSeries = zeros(0, numel(state.currentQ));
playbackSegmentNames = cell(0, 1);
qPrev = state.currentQ;
dqPrev = zeros(1, numel(state.currentQ));
prevTform = [];
for i = 1:size(targetPlan.points, 1)
    seedList = buildIkSeedList(state.robot, qPrev, state.initialJointPosition);
    [qSol, ~] = solveTcpPoseIk(state, targetPlan.tforms{i}, seedList, qPrev, dqPrev);
    appendSeries = qSol;
    if ~isempty(prevTform)
        targetPosStep = norm(targetPlan.tforms{i}(1:3, 4) - prevTform(1:3, 4));
        targetRotStep = rotationDistance(targetPlan.tforms{i}(1:3, 1:3), prevTform(1:3, 1:3));
        qStep = norm(wrapJointDelta(qSol - qPrev));
        if qStep > deg2rad(20.0) && targetPosStep < 0.004 && targetRotStep < deg2rad(6.0)
            try
                appendSeries = solveLocalPlaybackSubsteps(state, prevTform, targetPlan.tforms{i}, qPrev, dqPrev);
            catch
                appendSeries = qSol;
            end
        end
    end
    playbackQSeries = [playbackQSeries; appendSeries]; %#ok<AGROW>
    playbackSegmentNames = [playbackSegmentNames; repmat(targetPlan.segmentNames(i), size(appendSeries, 1), 1)]; %#ok<AGROW>
    dqPrev = wrapJointDelta(appendSeries(end, :) - qPrev);
    qPrev = appendSeries(end, :);
    prevTform = targetPlan.tforms{i};
    if i == 1 || i == size(targetPlan.points, 1) || mod(i, 20) == 0
        frac = 0.38 + 0.22 * (i / max(size(targetPlan.points, 1), 1));
        state = updateProgressStatus(state, sprintf('正在生成连续回放轨迹 %d/%d ...', i, size(targetPlan.points, 1)), frac);
    end
end
end

function qSeries = solveLocalPlaybackSubsteps(state, T0, T1, qPrev, dqPrev)
posStep = norm(T1(1:3, 4) - T0(1:3, 4));
rotStep = rotationDistance(T0(1:3, 1:3), T1(1:3, 1:3));
nSub = max([6, ceil(posStep / 0.0015), ceil(rotStep / deg2rad(2.0))]);
nSub = min(nSub, 24);
qSeries = zeros(nSub, numel(qPrev));
qLast = qPrev;
dqLast = dqPrev;
for k = 1:nSub
    alpha = k / nSub;
    p = ((1 - alpha) * T0(1:3, 4) + alpha * T1(1:3, 4)).';
    R = slerpRotation(T0(1:3, 1:3), T1(1:3, 1:3), alpha);
    T = rtfg_utils('rt2tform', R, p);
    qSeries(k, :) = solveTcpPoseIkLocal(state, T, qLast, dqLast);
    dqLast = wrapJointDelta(qSeries(k, :) - qLast);
    qLast = qSeries(k, :);
end
end

function qSol = solveTcpPoseIkLocal(state, targetTform, qPrev, dqPrev, clearanceThreshold)
ikSolver = state.ikSolver;
weightSchedule = {
    [1 1 1 0.20 0.20 0.20], deg2rad(25)
    [1 1 1 0.10 0.10 0.10], deg2rad(35)
    [1 1 1 0.03 0.03 0.03], deg2rad(55)
    [1 1 1 0.00 0.00 0.00], inf};
bestCandidate = [];
bestCost = inf;
robot = ikSolver.RigidBodyTree;
if nargin < 5 || isempty(clearanceThreshold)
    clearanceThreshold = state.collisionConfig.clearanceThreshold;
end
for i = 1:size(weightSchedule, 1)
    weights = weightSchedule{i, 1};
    orientationLimit = weightSchedule{i, 2};
    [qTry, ~] = ikSolver('sensor_shovel_tcp', targetTform, weights, qPrev);
    if any(~isfinite(qTry))
        continue;
    end
    qTry = alignEquivalentConfiguration(robot, qTry, qPrev, dqPrev);
    [positionError, orientationError] = evaluateTcpPoseError(robot, qTry, targetTform);
    collisionMetrics = rtfg_collision('evaluateConfiguration', state, qTry);
    minClearance = min([collisionMetrics.minSelfClearance, collisionMetrics.minToolBodyClearance, collisionMetrics.minToolBasinClearance]);
    if positionError <= 3e-2 && orientationError <= orientationLimit && minClearance >= clearanceThreshold
        candidateCost = continuityCost(qTry, qPrev, dqPrev);
        if candidateCost < bestCost
            bestCost = candidateCost;
            bestCandidate = qTry;
        end
    end
end
if isempty(bestCandidate)
    error('main_realtime_trajectory_fit_gui:LocalPlaybackStepFailed', '局部连续回放求解失败');
end
qSol = bestCandidate;
end

function nSeg = estimatePlaybackSegmentSamples(robot, q0, q1)
dq = wrapJointDelta(q1 - q0);
jointStep = norm(dq);
T0 = getTransform(robot, q0, 'sensor_shovel_tcp');
T1 = getTransform(robot, q1, 'sensor_shovel_tcp');
posStep = norm(T1(1:3, 4) - T0(1:3, 4));
rotStep = rotationDistance(T0(1:3, 1:3), T1(1:3, 1:3));
nSeg = 1 + max([ ...
    2, ...
    ceil(jointStep / deg2rad(0.70)), ...
    ceil(posStep / 0.0030), ...
    ceil(rotStep / deg2rad(0.30))]);
nSeg = min(max(nSeg, 4), 64);
end

function metrics = computePreviewMetrics(robot, targetPlan, anchorPlan, anchorQSeries, playbackQSeries, startQ)
metrics = struct();
metrics.anchorCount = size(anchorQSeries, 1);
metrics.playbackCount = size(playbackQSeries, 1);
metrics.maxTargetRotationDeltaDeg = 0.0;
metrics.maxActualRotationDeltaDeg = 0.0;
metrics.maxAnchorJointStepDegNorm = 0.0;
metrics.maxPlaybackJointStepDegNorm = 0.0;
metrics.moveToTrackTcpDeltaDeg = 0.0;
metrics.moveToTrackJointDeltaDegNorm = 0.0;

if isfield(anchorPlan, 'tforms') && numel(anchorPlan.tforms) >= 2
    metrics.maxTargetRotationDeltaDeg = rad2deg(maxSuccessiveRotationDelta(anchorPlan.tforms));
end

if ~isempty(anchorQSeries)
    anchorWithStart = [startQ; anchorQSeries];
    metrics.maxAnchorJointStepDegNorm = rad2deg(maxSuccessiveJointStep(anchorWithStart));
    metrics.moveToTrackJointDeltaDegNorm = rad2deg(norm(wrapJointDelta(anchorWithStart(2, :) - anchorWithStart(1, :))));
    Tstart = getTransform(robot, startQ, 'sensor_shovel_tcp');
    Tfirst = getTransform(robot, anchorQSeries(1, :), 'sensor_shovel_tcp');
    metrics.moveToTrackTcpDeltaDeg = rad2deg(rotationDistance(Tstart(1:3, 1:3), Tfirst(1:3, 1:3)));
end

if ~isempty(playbackQSeries)
    actualTforms = cell(size(playbackQSeries, 1), 1);
    for i = 1:size(playbackQSeries, 1)
        actualTforms{i} = getTransform(robot, playbackQSeries(i, :), 'sensor_shovel_tcp');
    end
    metrics.maxActualRotationDeltaDeg = rad2deg(maxSuccessiveRotationDelta(actualTforms));
    metrics.maxPlaybackJointStepDegNorm = rad2deg(maxSuccessiveJointStep(playbackQSeries));
end

if nargin >= 2 && isfield(targetPlan, 'sampleIndices') && ~isempty(targetPlan.sampleIndices)
    metrics.anchorSampleIndices = anchorPlan.sampleIndices;
else
    metrics.anchorSampleIndices = [];
end
end

function delta = maxSuccessiveRotationDelta(tforms)
delta = 0.0;
for i = 1:numel(tforms) - 1
    R0 = tforms{i}(1:3, 1:3);
    R1 = tforms{i + 1}(1:3, 1:3);
    delta = max(delta, rotationDistance(R0, R1));
end
end

function step = maxSuccessiveJointStep(qSeries)
step = 0.0;
for i = 1:size(qSeries, 1) - 1
    dq = wrapJointDelta(qSeries(i + 1, :) - qSeries(i, :));
    step = max(step, norm(dq));
end
end

function statusText = composeKinematicsStatus(baseText, metrics)
statusText = baseText;
if isempty(metrics) || ~isstruct(metrics) || ~isfield(metrics, 'maxPlaybackJointStepDegNorm')
    return;
end
statusText = sprintf('%s。播放层 max TCP 姿态步长 %.3f deg；播放层 max 关节步长 %.3f deg；锚点层 max 关节步长 %.3f deg', ...
    statusText, metrics.maxActualRotationDeltaDeg, metrics.maxPlaybackJointStepDegNorm, metrics.maxAnchorJointStepDegNorm);
end

function cost = continuityCost(qCandidate, qPrev, dqPrev)
dqPos = wrapJointDelta(qCandidate - qPrev);
dqVel = dqPos - dqPrev;
cost = norm(dqPos) + 0.65 * norm(dqVel);
end

function stride = choosePlaybackRenderStride(nPts)
if nPts >= 900
    stride = 3;
elseif nPts >= 600
    stride = 2;
else
    stride = 1;
end
end

function qAligned = alignEquivalentConfiguration(robot, qIn, qPrev, dqPrev)
qAligned = qIn;
targetRef = qPrev + dqPrev;
jointIndex = 0;
for i = 1:numel(robot.Bodies)
    joint = robot.Bodies{i}.Joint;
    if strcmp(joint.Type, 'fixed')
        continue;
    end
    jointIndex = jointIndex + 1;
    if ~strcmp(joint.Type, 'revolute')
        continue;
    end
    candidates = qAligned(jointIndex) + 2 * pi * (-2:2);
    lower = joint.PositionLimits(1);
    upper = joint.PositionLimits(2);
    if all(isfinite([lower, upper]))
        inRange = candidates >= lower - 1e-9 & candidates <= upper + 1e-9;
        if any(inRange)
            candidates = candidates(inRange);
        end
    end
    [~, bestIdx] = min(abs(candidates - targetRef(jointIndex)));
    qAligned(jointIndex) = candidates(bestIdx);
end
end

function dq = wrapJointDelta(dqIn)
dq = atan2(sin(dqIn), cos(dqIn));
end

function angle = rotationDistance(R0, R1)
R = R0' * R1;
angle = acos(max(-1.0, min(1.0, (trace(R) - 1) / 2)));
end
