function varargout = rtfg_collision(action, varargin)
switch action
    case 'buildConfig'
        varargout{1} = buildConfig();
    case 'emptyResults'
        varargout{1} = emptyResults();
    case 'buildEnvironment'
        varargout{1} = buildEnvironment(varargin{1});
    case 'evaluateConfiguration'
        varargout{1} = evaluateConfiguration(varargin{1}, varargin{2});
    case 'evaluateTrajectory'
        varargout{1} = evaluateTrajectory(varargin{1}, varargin{2}, varargin{3});
    case 'summarizeResults'
        varargout{1} = summarizeResults(varargin{1}, varargin{2});
    otherwise
        error('rtfg_collision:UnknownAction', 'Unknown action: %s', action);
end
end

function config = buildConfig()
config = struct();
config.robotBaseName = 'ur10';
config.toolBodies = {'sensor_shovel', 'sensor_shovel_tcp'};
config.skippedSelfCollisions = 'adjacent';
config.denseSamplesPerInterval = 2;
config.collisionEpsilon = 1e-6;
config.clearanceThreshold = 2e-3;
config.highlightColor = [0.95 0.10 0.10];
config.markerColor = [0.95 0.10 0.10];
config.markerSize = 26;
config.firstMarkerSize = 64;
end

function results = emptyResults()
results = struct();
results.hasCollision = false;
results.collisionPoints = zeros(0, 3);
results.collisionTypes = cell(0, 1);
results.collisionObjects = cell(0, 1);
results.segmentNames = cell(0, 1);
results.sampleIndices = zeros(0, 1);
results.trajectoryIndices = zeros(0, 1);
results.hitBasinNames = cell(0, 1);
results.firstCollision = struct([]);
results.minSelfClearance = inf;
results.minToolBodyClearance = inf;
results.minToolBasinClearance = inf;
results.minSelfObject = '';
results.minToolBodyObject = '';
results.minToolBasinObject = '';
results.minSelfTrajectoryIndex = NaN;
results.minToolBodyTrajectoryIndex = NaN;
results.minToolBasinTrajectoryIndex = NaN;
results.minSelfSegmentName = '';
results.minToolBodySegmentName = '';
results.minToolBasinSegmentName = '';
end

function env = buildEnvironment(state)
env = struct();
if isempty(state.robot) || isempty(state.collisionRobot)
    return;
end

TbaseToRobot = getTransform(state.robot, state.currentQ, state.collisionConfig.robotBaseName);
TrobotToBase = invertRigidTform(TbaseToRobot);
basinBoxes = buildBasinBoxes(state.pose);
worldObjects = cell(numel(basinBoxes), 1);
worldNames = cell(numel(basinBoxes), 1);
for i = 1:numel(basinBoxes)
    box = collisionBox(basinBoxes(i).size(1), basinBoxes(i).size(2), basinBoxes(i).size(3));
    box.Pose = TrobotToBase * basinBoxes(i).poseWorld;
    basinBoxes(i).poseRobot = box.Pose;
    worldObjects{i} = box;
    worldNames{i} = basinBoxes(i).name;
end

env.robotBasePose = TbaseToRobot;
env.worldObjects = worldObjects;
env.worldNames = worldNames;
env.basinBoxes = basinBoxes;
end

function results = evaluateTrajectory(state, qSeries, segmentNames)
results = emptyResults();
if ~state.collisionEnabled || isempty(qSeries)
    return;
end

[denseQ, denseSourceIdx, denseSegmentNames] = densifyJointSeries(qSeries, segmentNames, state.collisionConfig.denseSamplesPerInterval);

nSamples = size(denseQ, 1);
collisionPoints = zeros(0, 3);
collisionTypes = cell(0, 1);
collisionObjects = cell(0, 1);
segmentOut = cell(0, 1);
sampleIdxOut = zeros(0, 1);
trajIdxOut = zeros(0, 1);
hitBasin = {};
firstCollision = [];

for i = 1:nSamples
    q = denseQ(i, :);
    metrics = evaluateConfiguration(state, q);
    results = updateMinimumClearances(results, metrics, denseSourceIdx(i), denseSegmentNames{i});
    event = metrics.firstViolation;
    if isempty(event)
        continue;
    end

    tcpPoint = tform2trvec(getTransform(state.robot, q, 'sensor_shovel_tcp'));
    collisionPoints(end + 1, :) = tcpPoint; %#ok<AGROW>
    collisionTypes{end + 1, 1} = event.type; %#ok<AGROW>
    collisionObjects{end + 1, 1} = event.objectName; %#ok<AGROW>
    segmentOut{end + 1, 1} = denseSegmentNames{i}; %#ok<AGROW>
    sampleIdxOut(end + 1, 1) = i; %#ok<AGROW>
    trajIdxOut(end + 1, 1) = denseSourceIdx(i); %#ok<AGROW>
    if strcmp(event.type, 'tool_basin')
        hitBasin{end + 1, 1} = event.objectName; %#ok<AGROW>
    end
    if isempty(firstCollision)
        firstCollision = struct( ...
            'sampleIndex', i, ...
            'trajectoryIndex', denseSourceIdx(i), ...
            'segmentName', denseSegmentNames{i}, ...
            'type', event.type, ...
            'objectName', event.objectName, ...
            'point', tcpPoint, ...
            'clearance', event.clearance);
    end
end

results.hasCollision = ~isempty(collisionTypes);
results.collisionPoints = collisionPoints;
results.collisionTypes = collisionTypes;
results.collisionObjects = collisionObjects;
results.segmentNames = segmentOut;
results.sampleIndices = sampleIdxOut;
results.trajectoryIndices = trajIdxOut;
results.hitBasinNames = unique(hitBasin, 'stable');
if ~isempty(firstCollision)
    results.firstCollision = firstCollision;
end
end

function metrics = evaluateConfiguration(state, q)
metrics = struct();
metrics.minSelfClearance = inf;
metrics.minToolBodyClearance = inf;
metrics.minToolBasinClearance = inf;
metrics.minSelfObject = '';
metrics.minToolBodyObject = '';
metrics.minToolBasinObject = '';
metrics.violations = struct('type', {}, 'objectName', {}, 'clearance', {});
metrics.firstViolation = [];

collisionRobot = state.collisionRobot;
config = state.collisionConfig;
robotNames = [{collisionRobot.BaseName}, collisionRobot.BodyNames];
toolRows = find(ismember(robotNames, config.toolBodies));

[metrics.minSelfClearance, metrics.minToolBodyClearance, ...
    metrics.minSelfObject, metrics.minToolBodyObject] = ...
    findRobotMinimumClearances(collisionRobot, q, robotNames, config);

[metrics.minToolBasinClearance, metrics.minToolBasinObject] = ...
    findToolBasinMinimumClearance(collisionRobot, q, toolRows, state.collisionEnv);

if metrics.minToolBasinClearance < config.clearanceThreshold
    metrics.violations(end + 1) = struct( ...
        'type', 'tool_basin', ...
        'objectName', metrics.minToolBasinObject, ...
        'clearance', metrics.minToolBasinClearance); %#ok<AGROW>
end
if metrics.minToolBodyClearance < config.clearanceThreshold
    metrics.violations(end + 1) = struct( ...
        'type', 'tool_body', ...
        'objectName', metrics.minToolBodyObject, ...
        'clearance', metrics.minToolBodyClearance); %#ok<AGROW>
end
if metrics.minSelfClearance < config.clearanceThreshold
    metrics.violations(end + 1) = struct( ...
        'type', 'self', ...
        'objectName', metrics.minSelfObject, ...
        'clearance', metrics.minSelfClearance); %#ok<AGROW>
end

for i = 1:numel(metrics.violations)
    metrics.firstViolation = chooseHigherPriorityEvent(metrics.firstViolation, metrics.violations(i));
end
end

function text = summarizeResults(results, defaultText)
if nargin < 2
    defaultText = '';
end
summary = sprintf('最小 self 间隙 %.3f mm；最小 tool_body 间隙 %.3f mm；最小 tool_basin 间隙 %.3f mm', ...
    1000 * results.minSelfClearance, ...
    1000 * results.minToolBodyClearance, ...
    1000 * results.minToolBasinClearance);
if ~results.hasCollision
    if strlength(string(defaultText)) == 0
        text = summary;
    else
        text = sprintf('%s。%s。', defaultText, summary);
    end
    return;
end

fc = results.firstCollision;
suffix = sprintf('第 %d 点 [%s] clearance 仅 %.3f mm: %s', ...
    fc.trajectoryIndex, fc.segmentName, 1000 * fc.clearance, fc.objectName);
if strlength(string(defaultText)) == 0
    text = sprintf('%s。%s', suffix, summary);
else
    text = sprintf('%s，但%s。%s', defaultText, suffix, summary);
end
end

function boxes = buildBasinBoxes(pose)
Tpose = rtfg_utils('rt2tform', rtfg_utils('poseRotationMatrix', pose), [pose.x pose.y pose.z]);

boxSpecs = {
    'block_basin_bottom', [0.37 0.50 0.003], [-0.135 0 0.25]
    'block_basin_front_wall', [0.37 0.003 0.18], [-0.135 0.2485 0.34]
    'block_basin_back_wall', [0.37 0.003 0.18], [-0.135 -0.2485 0.34]
    'block_basin_left_wall', [0.003 0.494 0.18], [-0.3185 0 0.34]
    'block_basin_right_wall', [0.003 0.494 0.18], [0.0485 0 0.34]};

boxes = repmat(struct('name', '', 'size', [], 'poseWorld', [], 'poseRobot', []), size(boxSpecs, 1), 1);
for i = 1:size(boxSpecs, 1)
    boxes(i).name = boxSpecs{i, 1};
    boxes(i).size = boxSpecs{i, 2};
    boxes(i).poseWorld = Tpose * trvec2tform(boxSpecs{i, 3});
end
end

function [minSelf, minToolBody, minSelfObject, minToolBodyObject] = findRobotMinimumClearances(robot, q, robotNames, config)
minSelf = inf;
minToolBody = inf;
minSelfObject = '';
minToolBodyObject = '';
[~, separationDist] = checkCollision(robot, q, ...
    'Exhaustive', 'on', ...
    'IgnoreSelfCollision', 'off', ...
    'SkippedSelfCollisions', config.skippedSelfCollisions);

toolBodies = config.toolBodies;
nBodies = numel(robotNames);
for i = 1:nBodies-1
    for j = i+1:nBodies
        dist = symmetricSeparationDistance(separationDist, i, j);
        if ~isfinite(dist)
            continue;
        end
        nameA = robotNames{i};
        nameB = robotNames{j};
        aIsTool = any(strcmp(nameA, toolBodies));
        bIsTool = any(strcmp(nameB, toolBodies));
        if aIsTool && bIsTool
            continue;
        end
        if aIsTool || bIsTool
            if dist < minToolBody
                minToolBody = dist;
                minToolBodyObject = sprintf('%s <-> %s', nameA, nameB);
            end
        else
            if dist < minSelf
                minSelf = dist;
                minSelfObject = sprintf('%s <-> %s', nameA, nameB);
            end
        end
    end
end
end

function [minToolBasin, minToolBasinObject] = findToolBasinMinimumClearance(robot, q, toolRows, env)
minToolBasin = inf;
minToolBasinObject = '';
if isempty(env) || ~isfield(env, 'worldObjects') || isempty(env.worldObjects)
    return;
end

[~, separationDist] = checkCollision(robot, q, env.worldObjects, ...
    'Exhaustive', 'on', ...
    'IgnoreSelfCollision', 'on');

for i = 1:numel(toolRows)
    rowIdx = toolRows(i);
    for k = 1:numel(env.worldObjects)
        dist = separationDist(rowIdx, k);
        if isfinite(dist) && dist < minToolBasin
            minToolBasin = dist;
            minToolBasinObject = sprintf('%s <-> %s', robot.BodyNames{rowIdx - 1}, env.worldNames{k});
        end
    end
end
end

function event = chooseHigherPriorityEvent(firstEvent, secondEvent)
if isempty(firstEvent)
    event = secondEvent;
    return;
end
if isempty(secondEvent)
    event = firstEvent;
    return;
end
priority = struct('tool_basin', 3, 'tool_body', 2, 'self', 1);
if priority.(secondEvent.type) >= priority.(firstEvent.type)
    event = secondEvent;
else
    event = firstEvent;
end
end

function [denseQ, sourceIdx, segmentNames] = densifyJointSeries(qSeries, segmentNamesIn, nBetween)
if ischar(segmentNamesIn) || isstring(segmentNamesIn)
    segmentNamesIn = repmat(cellstr(string(segmentNamesIn)), size(qSeries, 1), 1);
elseif iscell(segmentNamesIn) && isscalar(segmentNamesIn)
    segmentNamesIn = repmat(segmentNamesIn, size(qSeries, 1), 1);
end

denseQ = qSeries(1, :);
sourceIdx = 1;
segmentNames = segmentNamesIn(1);
for i = 1:size(qSeries, 1)-1
    q0 = qSeries(i, :);
    q1 = qSeries(i + 1, :);
    for j = 1:nBetween
        alpha = j / (nBetween + 1);
        denseQ(end + 1, :) = (1 - alpha) * q0 + alpha * q1; %#ok<AGROW>
        sourceIdx(end + 1, 1) = i + 1; %#ok<AGROW>
        segmentNames{end + 1, 1} = segmentNamesIn{i + 1}; %#ok<AGROW>
    end
    denseQ(end + 1, :) = q1; %#ok<AGROW>
    sourceIdx(end + 1, 1) = i + 1; %#ok<AGROW>
    segmentNames{end + 1, 1} = segmentNamesIn{i + 1}; %#ok<AGROW>
end
end

function dist = symmetricSeparationDistance(separationDist, i, j)
vals = [separationDist(i, j), separationDist(j, i)];
vals = vals(isfinite(vals));
if isempty(vals)
    dist = inf;
else
    dist = min(vals);
end
end

function results = updateMinimumClearances(results, metrics, trajIdx, segmentName)
if metrics.minSelfClearance < results.minSelfClearance
    results.minSelfClearance = metrics.minSelfClearance;
    results.minSelfObject = metrics.minSelfObject;
    results.minSelfTrajectoryIndex = trajIdx;
    results.minSelfSegmentName = segmentName;
end
if metrics.minToolBodyClearance < results.minToolBodyClearance
    results.minToolBodyClearance = metrics.minToolBodyClearance;
    results.minToolBodyObject = metrics.minToolBodyObject;
    results.minToolBodyTrajectoryIndex = trajIdx;
    results.minToolBodySegmentName = segmentName;
end
if metrics.minToolBasinClearance < results.minToolBasinClearance
    results.minToolBasinClearance = metrics.minToolBasinClearance;
    results.minToolBasinObject = metrics.minToolBasinObject;
    results.minToolBasinTrajectoryIndex = trajIdx;
    results.minToolBasinSegmentName = segmentName;
end
end

function Tinv = invertRigidTform(T)
R = T(1:3, 1:3);
t = T(1:3, 4);
Tinv = eye(4);
Tinv(1:3, 1:3) = R.';
Tinv(1:3, 4) = -R.' * t;
end
