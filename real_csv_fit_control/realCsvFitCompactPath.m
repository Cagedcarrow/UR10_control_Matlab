function s = realCsvFitCompactPath(p)
if numel(p) <= 60
    s = p;
else
    s = ['...' p(end-56:end)];
end
end
