function [tVec, qCmd] = interp_joint_traj(waypointsQ, waypointTime, sampleTime, limits)
if size(waypointsQ,2) ~= 6
    error('waypointsQ 必须为N x 6。');
end
if numel(waypointTime) ~= size(waypointsQ,1)
    error('waypointTime 长度需与路径点数量一致。');
end
if any(diff(waypointTime) <= 0)
    error('waypointTime 必须严格递增。');
end

tVec = waypointTime(1):sampleTime:waypointTime(end);
qCmd = interp1(waypointTime(:), waypointsQ, tVec(:), 'pchip');

if nargin >= 4 && ~isempty(limits)
    qCmd = min(max(qCmd, limits(:,1)'), limits(:,2)');
end
end
