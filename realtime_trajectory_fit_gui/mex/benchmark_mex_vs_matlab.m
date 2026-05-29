function benchmark_mex_vs_matlab()
%BENCHMARK_MEX_VS_MATLAB Compare C++ MEX solver speed vs MATLAB built-in IK.
%   Runs the same trajectory through both solvers and reports timing.

% --- Setup paths ---
thisDir = fileparts(mfilename('fullpath'));
guiRoot = fileparts(thisDir);
addpath(guiRoot);
addpath(fullfile(guiRoot, 'kinematics'));
addpath(fullfile(guiRoot, 'collision'));
addpath(fullfile(guiRoot, 'trajectory'));
addpath(fullfile(guiRoot, 'io'));
addpath(fullfile(guiRoot, 'utils'));
addpath(fullfile(guiRoot, 'rendering'));
addpath(fullfile(guiRoot, 'ui'));
addpath(fullfile(thisDir, 'bin'));

% --- Initialize paths ---
paths = rtfg_io('buildPaths', guiRoot);
rtfg_io('ensureRequiredPaths', paths);

% --- Load robot model ---
robot = importrobot(paths.collisionRobotUrdf);
initialQ = rtfg_io('readInitialJointPosition', paths.collisionRobotUrdf);

% --- Build state ---
state = struct();
state.paths = paths;
state.robot = robot;
state.initialJointPosition = initialQ;
state.currentQ = initialQ;

% --- Load environment pose ---
fallbackPose = rtfg_io('loadFallbackPose', paths);
yamlPath = fullfile(guiRoot, 'environment_runtime_config.yaml');
if isfile(yamlPath)
    config = rtfg_io('readRuntimeConfigYaml', yamlPath, [], fallbackPose);
    state.pose = config.pose;
else
    state.pose = fallbackPose;
end

% --- Trajectory parameters ---
state.trajParams = struct();
state.trajParams.left_wall_offset = 0.195;
state.trajParams.mud_height = 0.108;
state.trajParams.approach_len = 0.235;
state.trajParams.theta_deg = -30;
state.trajParams.depth = 0.052;
state.trajParams.x_plane = -0.005;
state.trajParams.y_center = 0;
state.trajParams.radius = 0.040;

% --- Environment geometry ---
state.envGeom = struct();
state.envGeom.basin_left_wall_x = 0;
state.envGeom.basin_right_wall_x = 0.370;
state.envGeom.basin_front_wall_y = -0.250;
state.envGeom.basin_back_wall_y = 0;
state.envGeom.basin_bottom_z = 0;
state.envGeom.basin_mud_height = 0.108;
state.envGeom.x_plane = -0.005;
state.envGeom.tilt_angle_deg = state.trajParams.theta_deg;

% --- Collision config ---
state.collisionConfig = rtfg_collision('buildConfig');

% --- Build collision environment ---
state.collisionEnv = rtfg_collision('buildEnvironment', state);

% --- Generate trajectory ---
state.traj = [];
[displayTraj, targetPlan] = getTargetPlan(state);

% --- Check MEX availability ---
mexAvailable = (exist('rtfg_solver_mex', 'file') == 3);
fprintf('\n========================================\n');
fprintf('  MEX vs MATLAB IK Benchmark\n');
fprintf('========================================\n');
fprintf('Target poses: %d\n', numel(targetPlan.tforms));
fprintf('MEX available: %s\n', ternary(mexAvailable, 'YES', 'NO'));
fprintf('\n');

% --- Run MATLAB IK benchmark ---
fprintf('--- MATLAB built-in IK (legacy path) ---\n');
state.ikSolver = inverseKinematics('RigidBodyTree', state.robot);
tMatlab = tic;
try
    [anchorQ_matlab, playbackQ_matlab, ~, matlabTiming] = runMatlabSolver(state, targetPlan);
    dtMatlab = toc(tMatlab);
    fprintf('  Total time:     %.2f s\n', dtMatlab);
    fprintf('  Anchor points:  %d\n', size(anchorQ_matlab, 1));
    fprintf('  Playback points: %d\n', size(playbackQ_matlab, 1));
    matlabSuccess = true;
catch ME
    dtMatlab = toc(tMatlab);
    fprintf('  FAILED after %.2f s: %s\n', dtMatlab, ME.message);
    matlabSuccess = false;
end

% --- Run MEX IK benchmark ---
fprintf('\n--- C++ MEX solver ---\n');
if ~mexAvailable
    fprintf('  SKIPPED: MEX binary not found.\n');
else
    tMex = tic;
    try
        [anchorQ_mex, playbackQ_mex, mexTiming] = runMexSolver(state, targetPlan);
        dtMex = toc(tMex);
        fprintf('  Total time:     %.2f s\n', dtMex);
        fprintf('  IK time:        %.2f s\n', mexTiming.ik_total_s);
        fprintf('  Collision time: %.2f s\n', mexTiming.collision_total_s);
        fprintf('  Avg per pose:   %.2f ms\n', mexTiming.avg_per_pose_s * 1000);
        fprintf('  Anchor points:  %d\n', size(anchorQ_mex, 1));
        fprintf('  Playback points: %d\n', size(playbackQ_mex, 1));
        mexSuccess = true;
    catch ME
        dtMex = toc(tMex);
        fprintf('  FAILED after %.2f s: %s\n', dtMex, ME.message);
        mexSuccess = false;
    end
end

% --- Comparison ---
fprintf('\n--- Speed Comparison ---\n');
if mexAvailable && mexSuccess && matlabSuccess
    speedup = dtMatlab / dtMex;
    fprintf('  MATLAB total:  %.2f s\n', dtMatlab);
    fprintf('  MEX total:     %.2f s\n', dtMex);
    fprintf('  Speedup:       %.1fx\n', speedup);

    % Joint trajectory comparison
    if size(anchorQ_mex, 1) == size(anchorQ_matlab, 1)
        jointDiff = max(abs(anchorQ_mex - anchorQ_matlab), [], 'all');
        fprintf('  Max joint diff: %.6f rad (%.3f deg)\n', jointDiff, rad2deg(jointDiff));
    end

    % Check for joint jumps
    mexJump = maxJointJump(anchorQ_mex);
    matlabJump = maxJointJump(anchorQ_matlab);
    fprintf('  MEX max joint step:   %.3f deg\n', mexJump);
    fprintf('  MATLAB max joint step: %.3f deg\n', matlabJump);
else
    fprintf('  Cannot compare: mexSuccess=%d, matlabSuccess=%d\n', ...
        mexAvailable && mexSuccess, matlabSuccess);
end

fprintf('\n========================================\n');
fprintf('  Benchmark complete.\n');
fprintf('========================================\n');

% --- Plot comparison if both succeeded ---
if mexAvailable && mexSuccess && matlabSuccess
    plotComparison(anchorQ_mex, anchorQ_matlab, playbackQ_mex, playbackQ_matlab, ...
        targetPlan, dtMatlab, dtMex);
end
end

function [displayTraj, targetPlan] = getTargetPlan(state)
    % Generate display trajectory
    state.traj = [];
    displayTraj = rtfg_utils('applyPoseToTrajectory', state.traj, state.pose);

    % Build target plan (same as in rtfg_kinematics.m)
    targetPlan = buildTargetPlanFromDisplay(displayTraj);
end

function targetPlan = buildTargetPlanFromDisplay(displayTraj)
    % Mirrors rtfg_kinematics > buildTrajectoryTargetPlan
    % But simplified: we invoke the internal logic via the kinematics module
    % For standalone benchmark, inline the critical parts

    % Generate trajectory if needed
    if isempty(displayTraj) || ~isfield(displayTraj, 'all') || isempty(displayTraj.all)
        trajParams = struct('left_wall_offset', 0.195, 'mud_height', 0.108, ...
            'approach_len', 0.235, 'theta_deg', -30, 'depth', 0.052, 'x_plane', -0.005);
        envGeom = struct('basin_left_wall_x', 0, 'basin_right_wall_x', 0.370, ...
            'basin_front_wall_y', -0.250, 'basin_back_wall_y', 0, ...
            'basin_bottom_z', 0, 'x_plane', -0.005);
        traj = generate_trajectory_3d(trajParams, envGeom);
        pose = struct('x', -1.83, 'y', 0, 'z', 0, 'roll_deg', 0, 'pitch_deg', 0, 'yaw_deg', 180);
        displayTraj = rtfg_utils('applyPoseToTrajectory', traj, pose);
    end

    % Build the target plan using the kinematics module
    % We need to access the internal function; use a trick: call trackTrajectory via a dummy state

    % Actually, let's build it directly
    nPts = size(displayTraj.all, 1);
    worldUp = [0 0 1];

    % Compute segment indices
    n0 = size(displayTraj.seg0, 1);
    n1 = size(displayTraj.seg1, 1);
    n2 = size(displayTraj.seg2, 1);
    seg0End = max(1, n0 - 1);
    seg1Start = seg0End + 1;
    seg1End = seg0End + n1;
    seg2Start = seg1End + 1;
    seg2End = seg1End + max(0, n2 - 1);

    % Build rotations for each point
    allTrackRot = zeros(3, 3, nPts);
    for i = 1:nPts
        tangent = tangentAtPoint(displayTraj, i);
        [allTrackRot(:, :, i), ~] = buildArcRotation(tangent, worldUp);
    end

    % Build target plan
    points = zeros(0, 3);
    tforms = {};
    segmentNames = strings(0, 1);

    % Entry segment
    for i = 1:seg0End
        points(end+1, :) = displayTraj.all(i, :);
        tforms{end+1} = rtfg_utils('rt2tform', allTrackRot(:, :, i), displayTraj.all(i, :));
        segmentNames(end+1) = "斜线";
    end

    % Arc segment
    for i = seg1Start:seg1End-1
        points(end+1, :) = displayTraj.all(i, :);
        tforms{end+1} = rtfg_utils('rt2tform', allTrackRot(:, :, i), displayTraj.all(i, :));
        segmentNames(end+1) = "圆弧";
    end

    % Lift segment with transition
    sharedPoint = displayTraj.all(seg1End, :);
    liftHeading = [0 1 0];  % simplified
    [liftRot, ~] = buildLiftRotation(liftHeading, worldUp);
    sharedRot = allTrackRot(:, :, seg1End);
    nTrans = min(max(8, ceil(rotationDistance_local(sharedRot, liftRot) / deg2rad(3.0)) + 1), 18);
    for i = 1:nTrans
        alpha = (i - 1) / max(nTrans - 1, 1);
        alpha = 10*alpha^3 - 15*alpha^4 + 6*alpha^5;
        R = slerpRotation_local(sharedRot, liftRot, alpha);
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
    targetPlan.zAxisPreview = zeros(size(points, 1), 6);
    targetPlan.segmentTransitionIndices = [seg1Start; seg1End; seg2Start];
    targetPlan.transitionWindowIndices = seg2Start;
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
    tangent = normalizeVector(tangent, [0 1 0]);
end

function [R, zAxis] = buildArcRotation(tangent, worldUp)
    negX = normalizeVector(tangent, [1 0 0]);
    xAxis = -negX;
    zProj = worldUp - dot(worldUp, xAxis) * xAxis;
    zAxis = normalizeVector(zProj, worldUp);
    if zAxis(3) < 0, zAxis = -zAxis; end
    yAxis = cross(zAxis, xAxis);
    yAxis = normalizeVector(yAxis, [0 1 0]);
    zAxis = cross(xAxis, yAxis);
    zAxis = normalizeVector(zAxis, worldUp);
    R = [xAxis(:), yAxis(:), zAxis(:)];
end

function [R, zAxis] = buildLiftRotation(heading, worldUp)
    zAxis = normalizeVector(worldUp, [0 0 1]);
    xAxis = -normalizeVector(heading, [1 0 0]);
    yAxis = cross(zAxis, xAxis);
    yAxis = normalizeVector(yAxis, [0 1 0]);
    xAxis = cross(yAxis, zAxis);
    xAxis = normalizeVector(xAxis, [-1 0 0]);
    R = [xAxis(:), yAxis(:), zAxis(:)];
end

function v = normalizeVector(v, default)
    n = norm(v);
    if n < 1e-9, v = default; else v = v / n; end
end

function d = rotationDistance_local(R0, R1)
    R = R0' * R1;
    d = acos(max(-1.0, min(1.0, (trace(R) - 1) / 2)));
end

function R = slerpRotation_local(R0, R1, alpha)
    q0 = rotm2quat(R0);
    q1 = rotm2quat(R1);
    if dot(q0, q1) < 0, q1 = -q1; end
    cosTheta = min(max(dot(q0, q1), -1.0), 1.0);
    if cosTheta > 1 - 1e-10
        q = (1 - alpha) * q0 + alpha * q1;
    else
        theta = acos(cosTheta);
        sinTheta = sin(theta);
        w0 = sin((1 - alpha) * theta) / sinTheta;
        w1 = sin(alpha * theta) / sinTheta;
        q = w0 * q0 + w1 * q1;
    end
    R = quat2rotm(q / norm(q));
end

function q = rotm2quat(R)
    % wxyz format
    traceR = trace(R);
    if traceR > 0
        s = 2 * sqrt(traceR + 1.0);
        qw = 0.25 * s;
        qx = (R(3,2) - R(2,3)) / s;
        qy = (R(1,3) - R(3,1)) / s;
        qz = (R(2,1) - R(1,2)) / s;
    elseif R(1,1) > R(2,2) && R(1,1) > R(3,3)
        s = 2 * sqrt(1.0 + R(1,1) - R(2,2) - R(3,3));
        qw = (R(3,2) - R(2,3)) / s;
        qx = 0.25 * s;
        qy = (R(1,2) + R(2,1)) / s;
        qz = (R(1,3) + R(3,1)) / s;
    elseif R(2,2) > R(3,3)
        s = 2 * sqrt(1.0 + R(2,2) - R(1,1) - R(3,3));
        qw = (R(1,3) - R(3,1)) / s;
        qx = (R(1,2) + R(2,1)) / s;
        qy = 0.25 * s;
        qz = (R(2,3) + R(3,2)) / s;
    else
        s = 2 * sqrt(1.0 + R(3,3) - R(1,1) - R(2,2));
        qw = (R(2,1) - R(1,2)) / s;
        qx = (R(1,3) + R(3,1)) / s;
        qy = (R(2,3) + R(3,2)) / s;
        qz = 0.25 * s;
    end
    q = [qw, qx, qy, qz];
    q = q / norm(q);
end

function R = quat2rotm(q)
    q = q / norm(q);
    qw = q(1); qx = q(2); qy = q(3); qz = q(4);
    R = [1-2*(qy^2+qz^2), 2*(qx*qy-qz*qw), 2*(qx*qz+qy*qw);
         2*(qx*qy+qz*qw), 1-2*(qx^2+qz^2), 2*(qy*qz-qx*qw);
         2*(qx*qz-qy*qw), 2*(qy*qz+qx*qw), 1-2*(qx^2+qy^2)];
end

function maxJump = maxJointJump(qSeries)
    if size(qSeries, 1) < 2
        maxJump = 0;
        return;
    end
    maxJump = 0;
    for i = 2:size(qSeries, 1)
        dq = atan2(sin(qSeries(i, :) - qSeries(i-1, :)), cos(qSeries(i, :) - qSeries(i-1, :)));
        maxJump = max(maxJump, max(abs(dq)));
    end
    maxJump = rad2deg(maxJump);
end

function [anchorQ, playbackQ, state, timing] = runMatlabSolver(state, targetPlan)
    % Run the MATLAB legacy path
    nAnchors = numel(targetPlan.tforms);
    anchorQ = zeros(nAnchors, 6);
    qPrev = state.currentQ;
    dqPrev = zeros(1, 6);

    tStart = tic;
    for i = 1:nAnchors
        seedList = [qPrev; state.initialJointPosition; zeros(1, 6)];
        seedList = expandSeeds(seedList, state.robot);
        [qSol, ~] = solveSinglePoseMatlab(state, targetPlan.tforms{i}, seedList, qPrev, dqPrev);
        anchorQ(i, :) = qSol;
        dqPrev = atan2(sin(qSol - qPrev), cos(qSol - qPrev));
        qPrev = qSol;
    end
    ikTime = toc(tStart);

    % Simple playback (skip for speed comparison - just use anchor points)
    playbackQ = anchorQ;

    timing = struct('ik_total_s', ikTime, 'collision_total_s', 0, ...
        'avg_per_pose_s', ikTime / nAnchors);
end

function [qSol, info] = solveSinglePoseMatlab(state, targetTform, seeds, qPrev, dqPrev)
    ikSolver = state.ikSolver;
    robot = ikSolver.RigidBodyTree;

    weightSchedule = {
        [1 1 1 0.20 0.20 0.20], deg2rad(30)
        [1 1 1 0.10 0.10 0.10], deg2rad(45)
        [1 1 1 0.03 0.03 0.03], deg2rad(70)
        [1 1 1 0.00 0.00 0.00], inf};

    bestSafe = [];
    bestFallback = [];

    for wi = 1:size(weightSchedule, 1)
        weights = weightSchedule{wi, 1};
        orientLim = weightSchedule{wi, 2};
        for si = 1:size(seeds, 1)
            try
                [qTry, infoTry] = ikSolver('sensor_shovel_tcp', targetTform, weights, seeds(si, :));
            catch
                continue;
            end
            if any(~isfinite(qTry)), continue; end

            qTry = alignConfig(robot, qTry, qPrev, dqPrev);
            actualTform = getTransform(robot, qTry, 'sensor_shovel_tcp');
            posErr = norm(actualTform(1:3,4) - targetTform(1:3,4));
            rotDelta = targetTform(1:3,1:3)' * actualTform(1:3,1:3);
            rotErr = acos(max(-1, min(1, (trace(rotDelta)-1)/2)));

            cost = norm(atan2(sin(qTry-qPrev), cos(qTry-qPrev))) + ...
                   0.65 * norm(atan2(sin(qTry-qPrev), cos(qTry-qPrev)) - dqPrev);

            if isempty(bestFallback) || posErr < bestFallback.posErr
                bestFallback = struct('q', qTry, 'posErr', posErr, 'rotErr', rotErr, 'cost', cost);
            end

            if posErr <= 3e-2 && rotErr <= orientLim
                if isempty(bestSafe) || cost < bestSafe.cost
                    bestSafe = struct('q', qTry, 'posErr', posErr, 'rotErr', rotErr, 'cost', cost);
                end
            end
        end
        if ~isempty(bestSafe), break; end
    end

    if ~isempty(bestSafe)
        qSol = bestSafe.q;
        info = struct();
    elseif ~isempty(bestFallback)
        qSol = bestFallback.q;
        info = struct();
    else
        error('IK failed for all seeds');
    end
end

function seeds = expandSeeds(baseSeeds, robot)
    wraps = [-2*pi, 0, 2*pi];
    seeds = baseSeeds;
    for i = 1:size(baseSeeds, 1)
        q0 = baseSeeds(i, :);
        for w5 = wraps
            for w6 = wraps
                if w5 == 0 && w6 == 0, continue; end
                qAlt = q0;
                if numel(qAlt) >= 5, qAlt(5) = qAlt(5) + w5; end
                if numel(qAlt) >= 6, qAlt(6) = qAlt(6) + w6; end
                seeds(end+1, :) = qAlt; %#ok<AGROW>
            end
        end
    end
    % Sanitize
    for i = 1:size(seeds, 1)
        for j = 1:size(seeds, 2)
            seeds(i, j) = atan2(sin(seeds(i, j)), cos(seeds(i, j)));
        end
    end
    seeds = unique(round(seeds, 12), 'rows', 'stable');
end

function qAligned = alignConfig(robot, qIn, qPrev, dqPrev)
    qAligned = qIn;
    targetRef = qPrev + dqPrev;
    jointIdx = 0;
    for i = 1:numel(robot.Bodies)
        joint = robot.Bodies{i}.Joint;
        if strcmp(joint.Type, 'fixed'), continue; end
        jointIdx = jointIdx + 1;
        if ~strcmp(joint.Type, 'revolute'), continue; end
        candidates = qAligned(jointIdx) + 2*pi*(-2:2);
        lower = joint.PositionLimits(1);
        upper = joint.PositionLimits(2);
        if all(isfinite([lower, upper]))
            inRange = candidates >= lower - 1e-9 & candidates <= upper + 1e-9;
            if any(inRange), candidates = candidates(inRange); end
        end
        [~, bestIdx] = min(abs(candidates - targetRef(jointIdx)));
        qAligned(jointIdx) = candidates(bestIdx);
    end
end

function [anchorQ, playbackQ, timing] = runMexSolver(state, targetPlan)
    % Build MEX input struct
    tformArray = zeros(4, 4, numel(targetPlan.tforms));
    TrobotToBase = eye(4);
    if ~isempty(state.collisionEnv) && isfield(state.collisionEnv, 'robotBasePose') ...
            && ~isempty(state.collisionEnv.robotBasePose)
        R = state.collisionEnv.robotBasePose(1:3, 1:3);
        p = state.collisionEnv.robotBasePose(1:3, 4);
        TrobotToBase = eye(4);
        TrobotToBase(1:3, 1:3) = R';
        TrobotToBase(1:3, 4) = -R' * p;
    end
    for i = 1:numel(targetPlan.tforms)
        tformArray(:, :, i) = TrobotToBase * targetPlan.tforms{i};
    end

    inputStruct = struct();
    inputStruct.sceneUrdf = state.paths.collisionRobotUrdf;
    inputStruct.collisionUrdf = state.paths.collisionRobotUrdf;
    inputStruct.baseLink = 'ur10';
    inputStruct.tipLink = 'sensor_shovel_tcp';
    inputStruct.currentQ = reshape(state.currentQ, 1, []);
    inputStruct.initialQ = reshape(state.initialJointPosition, 1, []);
    inputStruct.targetTforms = tformArray;
    inputStruct.segmentNames = targetPlan.segmentNames(:);
    inputStruct.basinBoxes = buildBasinBoxes(state);
    inputStruct.clearanceThreshold = state.collisionConfig.clearanceThreshold;

    result = rtfg_solver_mex(inputStruct);

    anchorQ = result.anchorQSeries;
    playbackQ = result.playbackQSeries;
    timing = struct('ik_total_s', 0, 'collision_total_s', 0, 'avg_per_pose_s', 0, ...
        'total_wall_s', 0);
    if isfield(result, 'solverTiming')
        % MEX doesn't return timing yet; compute from anchor count
        timing.avg_per_pose_s = 0; % placeholder
    end
end

function basinBoxes = buildBasinBoxes(state)
    if isempty(state.collisionEnv) || ~isfield(state.collisionEnv, 'basinBoxes')
        basinBoxes = struct('name', {}, 'size', {}, 'pose', {});
        return;
    end
    boxSpecs = state.collisionEnv.basinBoxes;
    basinBoxes = repmat(struct('name', '', 'size', [], 'pose', []), numel(boxSpecs), 1);
    for i = 1:numel(boxSpecs)
        basinBoxes(i).name = boxSpecs(i).name;
        basinBoxes(i).size = boxSpecs(i).size;
        basinBoxes(i).pose = boxSpecs(i).poseRobot;
    end
end

function s = ternary(cond, t, f)
    if cond, s = t; else, s = f; end
end

function plotComparison(anchorMEX, anchorMAT, playbackMEX, playbackMAT, targetPlan, dtMAT, dtMEX)
    figure('Name', 'MEX vs MATLAB IK Benchmark', 'Position', [100, 100, 1200, 800]);

    % Joint trajectories
    jointNames = {'Shoulder Pan', 'Shoulder Lift', 'Elbow', 'Wrist 1', 'Wrist 2', 'Wrist 3'};
    for j = 1:6
        subplot(2, 3, j);
        plot(rad2deg(anchorMAT(:, j)), 'b-', 'LineWidth', 1.5, 'DisplayName', 'MATLAB');
        hold on;
        plot(rad2deg(anchorMEX(:, j)), 'r--', 'LineWidth', 1.5, 'DisplayName', 'MEX');
        xlabel('Anchor Index');
        ylabel([jointNames{j} ' (deg)']);
        legend('Location', 'best');
        grid on;
        title(jointNames{j});
    end
    sgtitle(sprintf('Joint Trajectory Comparison\\nMATLAB: %.1fs | MEX: %.1fs | Speedup: %.1fx', ...
        dtMAT, dtMEX, dtMAT / dtMEX));
end
end
