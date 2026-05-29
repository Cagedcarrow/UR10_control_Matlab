function qSeries = rtfg_quintic_joint_series(qStart, qEnd, nPts)
if nargin < 3 || isempty(nPts)
    nPts = 60;
end

t = linspace(0, 1, max(2, nPts)).';
s = 10 * t.^3 - 15 * t.^4 + 6 * t.^5;
dq = atan2(sin(qEnd - qStart), cos(qEnd - qStart));
qSeries = qStart + dq .* s;
end
