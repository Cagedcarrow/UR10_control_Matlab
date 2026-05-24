function [ok,msg] = realCsvFitCheckLimits(q, lim)
if isempty(q) || size(q,2) ~= 6 || any(~isfinite(q(:)))
    ok = false;
    msg = '轨迹包含非法关节角。';
    return;
end

badLow = q < lim(:,1)';
badHigh = q > lim(:,2)';
bad = badLow | badHigh;
if any(bad(:))
    [row,col] = find(bad,1);
    ok = false;
    msg = sprintf('第%d个轨迹点第%d轴超出软限位 [%.1f, %.1f] 度。', ...
        row, col, rad2deg(lim(col,1)), rad2deg(lim(col,2)));
else
    ok = true;
    msg = 'OK';
end
end
