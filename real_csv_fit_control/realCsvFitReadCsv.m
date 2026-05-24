function data = realCsvFitReadCsv(csvPath, defaultSampleTime)
if ~isfile(csvPath)
    error('未找到CSV: %s', csvPath);
end

tbl = readtable(csvPath);
q = localGetCols(tbl, 'Act_q');
dq = localGetCols(tbl, 'Act_qd');

n = min(size(q,1), size(dq,1));
if n < 5
    error('数据点太少，无法拟合。');
end

data = struct();
data.q = q(1:n,:);
data.dq = dq(1:n,:);
data.t = localMakeTimeVector(tbl, n, defaultSampleTime);
data.n = n;
end

function mat = localGetCols(tbl, prefix)
mat = zeros(height(tbl),6);
for j = 1:6
    col = [prefix num2str(j-1)];
    if ~ismember(col, tbl.Properties.VariableNames)
        error('CSV缺少列: %s', col);
    end
    mat(:,j) = tbl.(col);
end
end

function t = localMakeTimeVector(tbl, n, defaultSampleTime)
vars = tbl.Properties.VariableNames;
if ismember('Time', vars)
    raw = double(tbl.Time(1:n));
elseif ismember('epoch_time', vars)
    raw = double(tbl.epoch_time(1:n));
else
    raw = (0:n-1)' * defaultSampleTime;
end

raw = raw(:);
if numel(raw) ~= n || any(~isfinite(raw)) || numel(unique(raw)) < 2
    t = (0:n-1)' * defaultSampleTime;
    return;
end

t = raw - raw(1);
[tUnique, uniqueIdx] = unique(t, 'stable');
if numel(uniqueIdx) < n
    t = (0:n-1)' * median(diff(tUnique));
end

dt = diff(t);
if any(dt <= 0)
    positiveDt = dt(dt > 0 & isfinite(dt));
    if isempty(positiveDt)
        t = (0:n-1)' * defaultSampleTime;
    else
        t = (0:n-1)' * median(positiveDt);
    end
end
end
