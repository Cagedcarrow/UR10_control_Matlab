function ur10_realtime_sync(durationSec)
% UR10_REALTIME_SYNC
% Load local xacro-like URDF, connect to UR controller, and mirror joint
% motion in MATLAB in near real time.

clc;
if nargin < 1
    durationSec = inf;
end

robotIp = '10.160.9.21';
robotPort = 30003;              % UR realtime interface (preferred)
samplePeriod = 0.03;            % ~33 Hz rendering
xacroPath = 'E:/UR10_control/assembly/assembly.urdf.xacro';
meshRootName = 'meshes';

fprintf('[INFO] Robot IP: %s, Port: %d\n', robotIp, robotPort);
fprintf('[INFO] Xacro path: %s\n', xacroPath);

robot = loadLocalRobotFromXacro(xacroPath, meshRootName);

cfg = homeConfiguration(robot);
numJoints = numel(cfg);
fprintf('[INFO] Non-fixed joints in model: %d\n', numJoints);
if numJoints < 6
    error('[ERROR] UR model joints < 6, cannot map UR10 joints.');
end

fig = figure('Name', 'UR10 Realtime Sync', 'Color', 'w');
ax = axes('Parent', fig);
showHandle = show(robot, cfg, 'Visuals', 'on', 'Collisions', 'off', 'Frames', 'off', 'Parent', ax); %#ok<NASGU>
axis(ax, 'equal');
grid(ax, 'on');
view(ax, 135, 20);
title(ax, sprintf('UR10 Loading Stream @ %s:%d', robotIp, robotPort));
drawnow;
try
    tcpObj = tcpclient(robotIp, robotPort, 'Timeout', 2);
catch ME
    fprintf(2, '[ERROR] Failed to connect robot socket: %s\n', ME.message);
    rethrow(ME);
end
fprintf('[INFO] Connected to robot realtime stream.\n');

% Avoid unbounded queued bytes during long runs.
flush(tcpObj);

lastPrint = tic;
packetCount = 0;
runTimer = tic;
rxBuffer = zeros(0,1,'uint8');
lastQ = nan(1,6);
expectedPacketLen = 0;
resyncCount = 0;
emptyReadCount = 0;
bytesInTotal = 0;

% Joint-name mapping (UR stream q_actual -> URDF joints).
urJointNames = {'ur10_shoulder_pan','ur10_shoulder_lift','ur10_elbow', ...
    'ur10_wrist_1','ur10_wrist_2','ur10_wrist_3'};
cfgJointNames = {cfg.JointName};
jointCfgIdx = zeros(1,6);
for i = 1:6
    idx = find(strcmp(cfgJointNames, urJointNames{i}), 1);
    if isempty(idx)
        idx = i; % fallback
        fprintf('[WARN] Joint name %s not found, fallback to index %d.\n', urJointNames{i}, i);
    end
    jointCfgIdx(i) = idx;
end

% Force initial pose to match robot before first rendering.
qInit = [];
tInit = tic;
while isempty(qInit)
    [qInit, rxBuffer, ~, expectedPacketLen, ~] = ...
        readLatestUrQActualFromRealtime(tcpObj, rxBuffer, expectedPacketLen);
    if toc(tInit) > 10
        fprintf('[WARN] Initial joint read timeout. Start from current cfg and wait for stream update.\n');
        break;
    end
    pause(0.005);
end
if ~isempty(qInit)
    for i = 1:6
        cfg(jointCfgIdx(i)).JointPosition = qInit(i);
    end
    lastQ = qInit;
    fprintf('[INFO] Initial q(rad)=[%.3f %.3f %.3f %.3f %.3f %.3f]\n', qInit(1), qInit(2), qInit(3), qInit(4), qInit(5), qInit(6));
end

% Fallback: if 30003 has no initial stream, try 30002 once.
if isempty(qInit) && robotPort == 30003
    fprintf('[INFO] Trying fallback port 30002 for initial pose...\n');
    clear tcpObj
    robotPort = 30002;
    tcpObj = tcpclient(robotIp, robotPort, 'Timeout', 2);
    flush(tcpObj);
    rxBuffer = zeros(0,1,'uint8');
    expectedPacketLen = 0;
    tInit2 = tic;
    while isempty(qInit)
        [qInit, rxBuffer, ~, expectedPacketLen, ~] = ...
            readLatestUrQActualFromRealtime(tcpObj, rxBuffer, expectedPacketLen);
        if toc(tInit2) > 6
            break;
        end
        pause(0.005);
    end
    if ~isempty(qInit)
        for i = 1:6
            cfg(jointCfgIdx(i)).JointPosition = qInit(i);
        end
        lastQ = qInit;
        fprintf('[INFO] Fallback initial q(rad)=[%.3f %.3f %.3f %.3f %.3f %.3f]\n', qInit(1), qInit(2), qInit(3), qInit(4), qInit(5), qInit(6));
    else
        fprintf('[WARN] Fallback port 30002 also did not provide initial pose in time.\n');
    end
end

title(ax, sprintf('UR10 Live Pose @ %s:%d', robotIp, robotPort));
drawnow;

while isvalid(fig)
    if toc(runTimer) >= durationSec
        fprintf('[INFO] Reached durationSec=%.2f, exiting loop.\\n', durationSec);
        break;
    end
    try
        [q, rxBuffer, newPackets, expectedPacketLen, lostSync, bytesReadNow] = ...
            readLatestUrQActualFromRealtime(tcpObj, rxBuffer, expectedPacketLen);
        bytesInTotal = bytesInTotal + bytesReadNow;
        if lostSync
            resyncCount = resyncCount + 1;
        end
        if isempty(q)
            emptyReadCount = emptyReadCount + 1;
            if toc(lastPrint) > 2
                fprintf('[DEBUG] no-packet: emptyRead=%d, bytesInTotal=%d, rxBuffer=%d, pktLen=%d, resync=%d\n', ...
                    emptyReadCount, bytesInTotal, numel(rxBuffer), expectedPacketLen, resyncCount);
                lastPrint = tic;
            end
            pause(0.005);
            drawnow limitrate;
            continue;
        end
        emptyReadCount = 0;
        packetCount = packetCount + newPackets;

        % Map first 6 joints from robot to UR q_actual.
        for i = 1:6
            cfg(jointCfgIdx(i)).JointPosition = q(i);
        end

        showHandle = show(robot, cfg, 'Visuals', 'on', 'Collisions', 'off', ...
            'Frames', 'off', 'Parent', ax, 'PreservePlot', false, 'FastUpdate', true); %#ok<NASGU>

        if toc(lastPrint) > 2
            if any(isnan(lastQ))
                dqNorm = NaN;
            else
                dqNorm = norm(q - lastQ);
            end
            fprintf('[DEBUG] packets=%d, newPackets=%d, pktLen=%d, resync=%d, dqNorm=%.6f, q(rad)=[%.3f %.3f %.3f %.3f %.3f %.3f]\n', ...
                packetCount, newPackets, expectedPacketLen, resyncCount, dqNorm, q(1), q(2), q(3), q(4), q(5), q(6));
            lastPrint = tic;
        end
        lastQ = q;

        drawnow limitrate;
        pause(samplePeriod);
    catch ME
        fprintf(2, '[WARN] Stream read/update failed: %s\n', ME.message);
        pause(0.1);
    end
end

fprintf('[INFO] Figure closed, exiting realtime sync.\n');
end

function robot = loadLocalRobotFromXacro(xacroPath, meshRootName)
if ~isfile(xacroPath)
    error('[ERROR] File not found: %s', xacroPath);
end

assemblyDir = fileparts(xacroPath);
meshDir = fullfile(assemblyDir, meshRootName);
if ~isfolder(meshDir)
    warning('[WARN] Mesh directory missing: %s', meshDir);
end

raw = fileread(xacroPath);
raw = regexprep(raw, '<xacro:arg[^>]*/>\s*', '');
raw = strrep(raw, '$(arg mesh_root)', meshRootName);

tmpUrdf = fullfile(tempdir, 'assembly_preprocessed_realtime.urdf');
fid = fopen(tmpUrdf, 'w');
if fid < 0
    error('[ERROR] Cannot write temp URDF: %s', tmpUrdf);
end
cleanObj = onCleanup(@() fclose(fid)); %#ok<NASGU>
fwrite(fid, raw, 'char');

fprintf('[INFO] Temp URDF: %s\n', tmpUrdf);
robot = importrobot(tmpUrdf, 'DataFormat', 'struct', 'MeshPath', meshDir);
end

function [qLatest, rxBuffer, parsedPackets, expectedPacketLen, lostSync, bytesReadNow] = ...
    readLatestUrQActualFromRealtime(tcpObj, rxBuffer, expectedPacketLen)
% Read all available bytes, parse complete realtime packets, and return
% q_actual from the most recent complete packet.

qLatest = [];
parsedPackets = 0;
lostSync = false;
bytesReadNow = 0;

avail = tcpObj.NumBytesAvailable;
if avail > 0
    newBytes = read(tcpObj, avail, 'uint8');
    rxBuffer = [rxBuffer; newBytes(:)]; %#ok<AGROW>
    bytesReadNow = numel(newBytes);
end

validLens = [1220, 1116, 1108];

while numel(rxBuffer) >= 4
    if expectedPacketLen <= 0
        [syncIdx, foundLen] = findSyncHeader(rxBuffer, validLens);
        if syncIdx < 1
            % Keep only tail bytes needed for future header detection.
            if numel(rxBuffer) > 3
                rxBuffer = rxBuffer(end-2:end);
            end
            break;
        end
        if syncIdx > 1
            rxBuffer = rxBuffer(syncIdx:end);
        end
        expectedPacketLen = foundLen;
    end

    packetLen = beInt32(rxBuffer(1:4));
    if packetLen ~= expectedPacketLen
        % Lost alignment, try to resync on next cycle.
        rxBuffer = rxBuffer(2:end);
        expectedPacketLen = 0;
        lostSync = true;
        continue;
    end

    if numel(rxBuffer) < packetLen
        % Incomplete packet, wait for more bytes next cycle.
        break;
    end

    packet = rxBuffer(1:packetLen);
    rxBuffer = rxBuffer(packetLen+1:end);
    qCand = parseQActualFromRealtimePacket(packet);
    if isValidQ(qCand)
        parsedPackets = parsedPackets + 1;
        qLatest = qCand;
    else
        % Data invalid -> treat as lost sync and rescan.
        expectedPacketLen = 0;
        lostSync = true;
    end
end
end

function q = parseQActualFromRealtimePacket(packet)
% Packet includes 4-byte length header, then data.
% q_actual starts at byte 253 in the full packet (1-based):
% 4-byte length + 8-byte time + 5 groups * 48 bytes = 252 bytes before q_actual.
qOffset = 253;  % 1-based byte index into packet
q = zeros(1, 6);
for i = 1:6
    i0 = qOffset + (i-1)*8;
    q(i) = beDouble(packet(i0:(i0+7)));
end
end

function tf = isValidQ(q)
tf = isvector(q) && numel(q) == 6 && all(isfinite(q)) && all(abs(q) < 20);
end

function [idx, foundLen] = findSyncHeader(rxBuffer, validLens)
idx = -1;
foundLen = 0;
n = numel(rxBuffer);
for k = 1:(n-3)
    L = beInt32(rxBuffer(k:k+3));
    if any(L == validLens)
        if n - k + 1 >= L
            qCand = parseQActualFromRealtimePacket(rxBuffer(k:k+L-1));
            if isValidQ(qCand)
                idx = k;
                foundLen = L;
                return;
            end
        end
    end
end
end

function v = beInt32(b)
if numel(b) ~= 4
    error('[ERROR] beInt32 input length must be 4.');
end
v = double(b(1))*16777216 + double(b(2))*65536 + double(b(3))*256 + double(b(4));
end

function d = beDouble(b)
if numel(b) ~= 8
    error('[ERROR] beDouble input length must be 8.');
end
d = typecast(uint8(b(end:-1:1)), 'double');
end
