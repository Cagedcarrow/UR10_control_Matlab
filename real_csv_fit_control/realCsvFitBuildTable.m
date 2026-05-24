function out = realCsvFitBuildTable(fit)
out = cell(6,4);
for j = 1:6
    out{j,1} = sprintf('J%d', j);
    out{j,2} = fit.qRmse(j);
    out{j,3} = fit.dqRmse(j);
    out{j,4} = fit.order;
end
end
