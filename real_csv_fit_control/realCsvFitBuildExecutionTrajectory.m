function [qExec, dtExec] = realCsvFitBuildExecutionTrajectory(t, qFit, cfg)
t = t(:);
if numel(t) ~= size(qFit,1)
    error('时间轴长度与轨迹点数不一致。');
end
if numel(t) < 2
    error('轨迹点太少，无法执行。');
end

mask = false(size(t));
mask(1) = true;
lastT = t(1);
for k = 2:numel(t)-1
    if (t(k) - lastT) * cfg.timeScale >= cfg.servojMinT
        mask(k) = true;
        lastT = t(k);
    end
end
mask(end) = true;

qExec = qFit(mask,:);
tExec = t(mask);
if size(qExec,1) > cfg.maxScriptPoints
    idx = unique(round(linspace(1,size(qExec,1),cfg.maxScriptPoints)));
    qExec = qExec(idx,:);
    tExec = tExec(idx);
end

dtExec = diff(tExec) * cfg.timeScale;
dtExec(dtExec < cfg.servojMinT) = cfg.servojMinT;
if isempty(dtExec)
    error('执行轨迹时间间隔为空。');
end
end
