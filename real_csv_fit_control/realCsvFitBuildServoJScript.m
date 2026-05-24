function script = realCsvFitBuildServoJScript(qExec, dtExec, cfg)
lines = cell(size(qExec,1)+3,1);
lines{1} = 'def csv_fit_traj():';
lines{2} = sprintf('  movej([%.10f,%.10f,%.10f,%.10f,%.10f,%.10f],a=%.5f,v=%.5f,t=%.5f,r=%.5f)', ...
    qExec(1,1),qExec(1,2),qExec(1,3),qExec(1,4),qExec(1,5),qExec(1,6), ...
    cfg.movejA,cfg.movejV,cfg.movejT,cfg.movejR);

for k = 2:size(qExec,1)
    dt = dtExec(k-1);
    lines{k+1} = sprintf(['  servoj([%.10f,%.10f,%.10f,%.10f,%.10f,%.10f],' ...
        't=%.5f,lookahead_time=%.5f,gain=%d)'], ...
        qExec(k,1),qExec(k,2),qExec(k,3),qExec(k,4),qExec(k,5),qExec(k,6), ...
        dt,cfg.servojLookahead,cfg.servojGain);
end

lines{end-1} = sprintf('  stopj(%.5f)', cfg.stopDecel);
lines{end} = 'end';
script = strjoin(lines, newline);
end
