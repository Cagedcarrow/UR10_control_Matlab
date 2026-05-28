function varargout = rtfg_utils(action, varargin)
switch action
    case 'applyPoseToTrajectory'
        varargout{1} = applyPoseToTrajectory(varargin{1}, varargin{2});
    case 'transformPointsByPose'
        varargout{1} = transformPointsByPose(varargin{1}, varargin{2});
    case 'rotateVectorsByPose'
        varargout{1} = rotateVectorsByPose(varargin{1}, varargin{2});
    case 'poseRotationMatrix'
        varargout{1} = poseRotationMatrix(varargin{1});
    case 'rt2tform'
        varargout{1} = rt2tform(varargin{1}, varargin{2});
    case 'normalizeRowVector'
        varargout{1} = normalizeRowVector(varargin{1}, varargin{2});
    case 'computeTcpPath'
        varargout{1} = computeTcpPath(varargin{1}, varargin{2});
    case 'getTcpPosition'
        varargout{1} = getTcpPosition(varargin{1}, varargin{2});
    case 'stringifyIkStatus'
        varargout{1} = stringifyIkStatus(varargin{1});
    case 'ternary'
        varargout{1} = ternary(varargin{1}, varargin{2}, varargin{3});
    otherwise
        error('rtfg_utils:UnknownAction', 'Unknown action: %s', action);
end
end

function trajOut = applyPoseToTrajectory(trajIn, pose)
trajOut = trajIn;
pointFields = {'all', 'seg0', 'seg1', 'seg2', 'pApproachStart', 'pEntry', 'pArcEnd', 'center'};
for i = 1:numel(pointFields)
    fieldName = pointFields{i};
    if isfield(trajOut, fieldName)
        trajOut.(fieldName) = transformPointsByPose(trajOut.(fieldName), pose);
    end
end
if isfield(trajOut, 'tangent3d')
    trajOut.tangent3d = rotateVectorsByPose(trajOut.tangent3d, pose);
end
end

function ptsOut = transformPointsByPose(ptsIn, pose)
if isempty(ptsIn)
    ptsOut = ptsIn;
    return;
end
isRowVector = isvector(ptsIn) && numel(ptsIn) == 3;
pts = reshape(ptsIn, [], 3);
R = poseRotationMatrix(pose);
t = [pose.x; pose.y; pose.z];
ptsOut = (R * pts.' + t).';
if isRowVector
    ptsOut = reshape(ptsOut, 1, 3);
end
end

function vecOut = rotateVectorsByPose(vecIn, pose)
if isempty(vecIn)
    vecOut = vecIn;
    return;
end
vec = reshape(vecIn, [], 3);
R = poseRotationMatrix(pose);
vecOut = (R * vec.').';
end

function R = poseRotationMatrix(pose)
roll = deg2rad(pose.rollDeg);
pitch = deg2rad(pose.pitchDeg);
yaw = deg2rad(pose.yawDeg);

cr = cos(roll); sr = sin(roll);
cp = cos(pitch); sp = sin(pitch);
cy = cos(yaw); sy = sin(yaw);

Rx = [1 0 0; 0 cr -sr; 0 sr cr];
Ry = [cp 0 sp; 0 1 0; -sp 0 cp];
Rz = [cy -sy 0; sy cy 0; 0 0 1];
R = Rz * Ry * Rx;
end

function T = rt2tform(R, translation)
T = eye(4);
T(1:3, 1:3) = R;
T(1:3, 4) = translation(:);
end

function vec = normalizeRowVector(vecIn, fallback)
vec = reshape(vecIn, 1, []);
nv = norm(vec);
if nv < 1e-9
    vec = reshape(fallback, 1, []);
    nv = norm(vec);
end
vec = vec ./ max(nv, 1e-12);
end

function tcpPath = computeTcpPath(robot, qSeries)
tcpPath = zeros(size(qSeries, 1), 3);
for i = 1:size(qSeries, 1)
    tcpPath(i, :) = getTcpPosition(robot, qSeries(i, :));
end
end

function tcpPos = getTcpPosition(robot, q)
tcpTform = getTransform(robot, q, 'sensor_shovel_tcp');
tcpPos = tform2trvec(tcpTform);
end

function textOut = stringifyIkStatus(info)
textOut = 'unknown';
if isstruct(info)
    if isfield(info, 'SolveMode')
        textOut = char(string(info.SolveMode));
        return;
    end
    if isfield(info, 'Status')
        textOut = char(string(info.Status));
    elseif isfield(info, 'ExitFlag')
        textOut = char(string(info.ExitFlag));
    elseif isfield(info, 'Iterations')
        textOut = sprintf('iterations=%d', info.Iterations);
    end
elseif isstring(info) || ischar(info)
    textOut = char(string(info));
end
end

function out = ternary(cond, trueVal, falseVal)
if cond
    out = trueVal;
else
    out = falseVal;
end
end
