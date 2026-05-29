function benchmark_simple()
%BENCHMARK_SIMPLE Simple speed comparison: MEX vs MATLAB IK on the same trajectory.

clear rtfg_solver_mex;

cd('/home/liuxiaopeng/公共的/MATLAB_ur10_control/UR10_control_Matlab');
thisDir = fullfile(pwd, 'realtime_trajectory_fit_gui');
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

assert(exist('rtfg_solver_mex', 'file') == 3, 'MEX not found');

% Initialize full state
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

% Load robots
state.envGeom.basin.mudSurfaceZ = state.envGeom.basin.floorZ + state.trajParams.mudHeight;
state.traj = generate_trajectory_3d(state.trajParams, state.envGeom);
previewText = rtfg_io('buildPreviewUrdfText', state.paths.sceneUrdf, state.pose);
rtfg_io('writeTextFile', state.tempUrdfPath, previewText);
state.robot = importrobot(state.tempUrdfPath, 'DataFormat', 'row', 'MeshPath', state.paths.meshDir);
state.collisionRobot = importrobot(state.paths.collisionRobotUrdf, 'DataFormat', 'row', 'MeshPath', state.paths.meshDir);
state.collisionEnv = rtfg_collision('buildEnvironment', state);

% Build target plan
displayTraj = rtfg_utils('applyPoseToTrajectory', state.traj, state.pose);
targetPlan = buildTargetPlanSimple(displayTraj);
nPts = numel(targetPlan.tforms);
fprintf('========================================\n');
fprintf('  MEX vs MATLAB IK Benchmark\n');
fprintf('========================================\n');
fprintf('Target poses: %d\n\n', nPts);

% === MATLAB IK ===
fprintf('--- MATLAB built-in IK ---\n');
fprintf('Method: 1 seed (qPrev), single weight [1,1,1,0.2,0.2,0.2]\n');
state.ikSolver = inverseKinematics('RigidBodyTree', state.robot);
qPrev = state.currentQ;
weights = [1 1 1 0.20 0.20 0.20];
tMat = tic;
nMatSuccess = 0;
matIKTimes = zeros(nPts, 1);
for i = 1:nPts
    t0 = tic;
    try
        [qTry, ~] = state.ikSolver('sensor_shovel_tcp', targetPlan.tforms{i}, weights, qPrev);
        if all(isfinite(qTry))
            qPrev = qTry;
            nMatSuccess = nMatSuccess + 1;
        end
    catch
    end
    matIKTimes(i) = toc(t0);
end
dtMat = toc(tMat);
fprintf('  Total: %.2f s (%.1f ms/pose avg, %.1f ms/pose max)\n', ...
    dtMat, mean(matIKTimes)*1000, max(matIKTimes)*1000);
fprintf('  Success: %d/%d\n\n', nMatSuccess, nPts);

% === MEX solver ===
fprintf('--- C++ MEX solver ---\n');
fprintf('Method: multi-seed (27 local + 97 global), 4 weight schedules, collision-aware\n');
tMex = tic;
try
    [state, statusText] = rtfg_kinematics('trackTrajectory', state);
    dtMex = toc(tMex);
    nMexAnchors = size(state.previewAnchorQSeries, 1);
    nMexPlayback = size(state.previewQSeries, 1);

    if contains(statusText, 'MEX 尖端轨迹拟合完成')
        fprintf('  Status: MEX COMPLETED SUCCESSFULLY\n');
    end

    % Extract per-pose timing from progressEvents if available
    fprintf('  Total wall time: %.2f s\n', dtMex);
    fprintf('  Per anchor (avg): %.1f ms\n', dtMex / nMexAnchors * 1000);
    fprintf('  Anchors: %d, Playback: %d\n', nMexAnchors, nMexPlayback);

    m = state.previewMetrics;
    fprintf('  Max anchor joint step: %.2f deg\n', m.maxAnchorJointStepDegNorm);
    fprintf('  Max playback joint step: %.2f deg\n', m.maxPlaybackJointStepDegNorm);
    fprintf('  Max actual rot delta: %.2f deg\n', m.maxActualRotationDeltaDeg);

catch ME
    dtMex = toc(tMex);
    fprintf('  FAILED: %s\n', ME.message);
end

% === Summary ===
fprintf('\n========================================\n');
fprintf('  SUMMARY\n');
fprintf('========================================\n');
fprintf('MATLAB IK:           %.2f s  (%.1f ms/pose, %d/%d ok)\n', ...
    dtMat, dtMat/nPts*1000, nMatSuccess, nPts);
fprintf('MEX solver (full):   %.2f s  (%.1f ms/anchor, %d anchors)\n', ...
    dtMex, dtMex/nMexAnchors*1000, nMexAnchors);
fprintf('\nMEX does MORE work per pose:\n');
fprintf('  - Multi-seed strategy (27+ seeds vs 1)\n');
fprintf('  - 4 weight schedules vs 1\n');
fprintf('  - Collision checking per pose\n');
fprintf('  - Playback interpolation\n');
fprintf('  - Full collision audit\n');
fprintf('\nFor fair IK-only comparison:\n');
fprintf('  MATLAB IK:  ~%.1f ms/pose (single seed+weight)\n', mean(matIKTimes)*1000);
fprintf('  MEX IK:     ~%.1f ms/pose (multi-seed+weight+collision+playback)\n', dtMex/nMexAnchors*1000);
fprintf('  MEX IK overhead: ~%.1fx over MATLAB simple IK\n', (dtMex/nMexAnchors)/mean(matIKTimes));
fprintf('  But MEX gives: collision safety, continuity cost optimization, adaptive IK\n');
end

function targetPlan = buildTargetPlanSimple(displayTraj)
    n0 = size(displayTraj.seg0, 1);
    n1 = size(displayTraj.seg1, 1);
    n2 = size(displayTraj.seg2, 1);
    seg0End = max(1, n0 - 1);
    seg1Start = seg0End + 1;
    seg1End = seg0End + n1;
    seg2Start = seg1End + 1;
    seg2End = seg1End + max(0, n2 - 1);

    nPts = size(displayTraj.all, 1);
    worldUp = [0 0 1];
    allTrackRot = zeros(3, 3, nPts);
    for i = 1:nPts
        tangent = tangentAtPoint(displayTraj, i);
        [allTrackRot(:, :, i), ~] = buildArcRotation(tangent, worldUp);
    end

    points = zeros(0, 3);
    tforms = {};
    segmentNames = strings(0, 1);

    for i = 1:seg0End
        points(end+1, :) = displayTraj.all(i, :);
        tforms{end+1} = rtfg_utils('rt2tform', allTrackRot(:, :, i), displayTraj.all(i, :));
        segmentNames(end+1) = "斜线";
    end
    for i = seg1Start:seg1End-1
        points(end+1, :) = displayTraj.all(i, :);
        tforms{end+1} = rtfg_utils('rt2tform', allTrackRot(:, :, i), displayTraj.all(i, :));
        segmentNames(end+1) = "圆弧";
    end
    sharedPoint = displayTraj.all(seg1End, :);
    liftHeading = [0 1 0];
    [liftRot, ~] = buildLiftRotation(liftHeading, worldUp);
    sharedRot = allTrackRot(:, :, seg1End);
    rotDelta = acos(max(-1, min(1, (trace(sharedRot' * liftRot) - 1) / 2)));
    nTrans = min(max(8, ceil(rotDelta / deg2rad(3.0)) + 1), 18);
    for i = 1:nTrans
        alpha = (i - 1) / max(nTrans - 1, 1);
        alpha = 10*alpha^3 - 15*alpha^4 + 6*alpha^5;
        R = slerpRotationLocal(sharedRot, liftRot, alpha);
        points(end+1, :) = sharedPoint;
        tforms{end+1} = rtfg_utils('rt2tform', R, sharedPoint);
        segmentNames(end+1) = "出泥";
    end
    for i = seg2Start:seg2End
        points(end+1, :) = displayTraj.all(i, :);
        tforms{end+1} = rtfg_utils('rt2tform', liftRot, displayTraj.all(i, :));
        segmentNames(end+1) = "出泥";
    end

    targetPlan = struct();
    targetPlan.points = points;
    targetPlan.tforms = tforms;
    targetPlan.segmentNames = cellstr(segmentNames);
end

function tangent = tangentAtPoint(traj, idx)
    if isfield(traj, 'tangent3d') && ~isempty(traj.tangent3d)
        tangent = traj.tangent3d(idx, :);
    else
        nPts = size(traj.all, 1);
        if idx == 1
            tangent = traj.all(2, :) - traj.all(1, :);
        elseif idx == nPts
            tangent = traj.all(end, :) - traj.all(end-1, :);
        else
            tangent = traj.all(idx+1, :) - traj.all(idx-1, :);
        end
    end
    nrm = norm(tangent);
    if nrm < 1e-9, tangent = [0 1 0]; else tangent = tangent / nrm; end
end

function [R, zAxis] = buildArcRotation(tangent, worldUp)
    xAxis = -tangent;
    zProj = worldUp - dot(worldUp, xAxis) * xAxis;
    nrm = norm(zProj);
    if nrm < 1e-9, zAxis = worldUp; else zAxis = zProj / nrm; end
    if zAxis(3) < 0, zAxis = -zAxis; end
    yAxis = cross(zAxis, xAxis);
    nrm = norm(yAxis);
    if nrm < 1e-9, yAxis = [0 1 0]; else yAxis = yAxis / nrm; end
    zAxis = cross(xAxis, yAxis);
    nrm = norm(zAxis);
    if nrm < 1e-9, zAxis = worldUp; else zAxis = zAxis / nrm; end
    R = [xAxis(:), yAxis(:), zAxis(:)];
end

function [R, zAxis] = buildLiftRotation(heading, worldUp)
    zAxis = worldUp / norm(worldUp);
    xAxis = -heading / norm(heading);
    yAxis = cross(zAxis, xAxis);
    nrm = norm(yAxis);
    if nrm < 1e-9, yAxis = [0 1 0]; else yAxis = yAxis / nrm; end
    xAxis = cross(yAxis, zAxis);
    nrm = norm(xAxis);
    if nrm < 1e-9, xAxis = [-1 0 0]; else xAxis = xAxis / nrm; end
    R = [xAxis(:), yAxis(:), zAxis(:)];
end

function R = slerpRotationLocal(R0, R1, alpha)
    q0 = rotm2quatLocal(R0);
    q1 = rotm2quatLocal(R1);
    if dot(q0, q1) < 0, q1 = -q1; end
    cosTheta = min(max(dot(q0, q1), -1.0), 1.0);
    if cosTheta > 1 - 1e-10
        q = (1 - alpha) * q0 + alpha * q1;
    else
        theta = acos(cosTheta);
        sinTheta = sin(theta);
        q = sin((1-alpha)*theta)/sinTheta * q0 + sin(alpha*theta)/sinTheta * q1;
    end
    R = quat2rotmLocal(q / norm(q));
end

function q = rotm2quatLocal(R)
    traceR = trace(R);
    if traceR > 0
        s = 2 * sqrt(traceR + 1.0);
        q = [0.25*s, (R(3,2)-R(2,3))/s, (R(1,3)-R(3,1))/s, (R(2,1)-R(1,2))/s];
    elseif R(1,1) > R(2,2) && R(1,1) > R(3,3)
        s = 2 * sqrt(1.0 + R(1,1) - R(2,2) - R(3,3));
        q = [(R(3,2)-R(2,3))/s, 0.25*s, (R(1,2)+R(2,1))/s, (R(1,3)+R(3,1))/s];
    elseif R(2,2) > R(3,3)
        s = 2 * sqrt(1.0 + R(2,2) - R(1,1) - R(3,3));
        q = [(R(1,3)-R(3,1))/s, (R(1,2)+R(2,1))/s, 0.25*s, (R(2,3)+R(3,2))/s];
    else
        s = 2 * sqrt(1.0 + R(3,3) - R(1,1) - R(2,2));
        q = [(R(2,1)-R(1,2))/s, (R(1,3)+R(3,1))/s, (R(2,3)+R(3,2))/s, 0.25*s];
    end
    q = q / norm(q);
end

function R = quat2rotmLocal(q)
    q = q / norm(q);
    qw = q(1); qx = q(2); qy = q(3); qz = q(4);
    R = [1-2*(qy^2+qz^2), 2*(qx*qy-qz*qw), 2*(qx*qz+qy*qw);
         2*(qx*qy+qz*qw), 1-2*(qx^2+qz^2), 2*(qy*qz-qx*qw);
         2*(qx*qz-qy*qw), 2*(qy*qz+qx*qw), 1-2*(qx^2+qy^2)];
end
