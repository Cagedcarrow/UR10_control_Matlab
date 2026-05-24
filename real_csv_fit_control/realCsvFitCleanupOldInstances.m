function realCsvFitCleanupOldInstances()
oldTimers = timerfindall('Tag','UR10_REAL_CSV_FIT_LOOP');
oldTimers = [oldTimers; timerfindall('Tag','UR10_REAL_CSV_FIT_PREVIEW')];
for i = 1:numel(oldTimers)
    try
        stop(oldTimers(i));
    catch
    end
    try
        delete(oldTimers(i));
    catch
    end
end

oldFigs = findall(groot,'Type','figure','Name','UR10 真机CSV轨迹拟合控制');
for i = 1:numel(oldFigs)
    try
        delete(oldFigs(i));
    catch
    end
end
end
