function result = evaluate_current_estimation_batch(dataRoot, cfg, opts)
% 批量评估: 估算电流 i_est vs 实测电流 Act_I*

if nargin < 1 || isempty(dataRoot)
    dataRoot = fullfile(fileparts(fileparts(mfilename('fullpath'))), 'data');
end
if nargin < 2 || isempty(cfg)
    cfg = init_virtual_ur10(fileparts(mfilename('fullpath')));
end
if nargin < 3
    opts = struct();
end
if ~isfield(opts,'singleFile')
    opts.singleFile = '';
end
if ~isfield(opts,'outputDir') || isempty(opts.outputDir)
    opts.outputDir = fullfile(cfg.baseDir, 'outputs', 'current_eval');
end
if ~isfield(opts,'maxPlots')
    opts.maxPlots = inf;
end

if ~exist(opts.outputDir,'dir')
    mkdir(opts.outputDir);
end

addpath(cfg.baseDir);
addpath(fullfile(cfg.baseDir,'utils'));
[robot, ~] = build_ur10_virtual_robot(cfg); %#ok<ASGLU>
mdl = create_or_load_virtual_simulink_model(robot, cfg);

csvFiles = discoverCsvFiles(dataRoot, opts.singleFile);
rows = [];
failed = {};
plotCount = 0;

for i = 1:numel(csvFiles)
    csvPath = csvFiles{i};
    try
        one = evaluateOneFile(mdl, cfg, csvPath, opts.outputDir, plotCount < opts.maxPlots);
        rows = [rows; one.metricsRow]; %#ok<AGROW>
        if plotCount < opts.maxPlots
            plotCount = plotCount + 1;
        end
    catch ME
        failed(end+1,:) = {csvPath, ME.message}; %#ok<AGROW>
    end
end

if isempty(rows)
    error('没有成功处理任何CSV文件。');
end

perFileTbl = struct2table(rows);
summaryPath = fullfile(opts.outputDir, 'summary.csv');
writetable(perFileTbl, summaryPath);

perJointOverall = computeOverall(perFileTbl);
worstCases = sortrows(perFileTbl, 'rmse_mean', 'descend');
worstCases = worstCases(1:min(10,height(worstCases)), :);

reportPath = fullfile(opts.outputDir, 'overall_report.md');
writeReport(reportPath, perFileTbl, perJointOverall, worstCases, failed, dataRoot);

result = struct();
result.per_file_metrics = perFileTbl;
result.per_joint_overall = perJointOverall;
result.worst_cases = worstCases;
result.output_dir = opts.outputDir;
result.summary_csv = summaryPath;
result.report_md = reportPath;
result.failed = failed;
end

function csvFiles = discoverCsvFiles(dataRoot, singleFile)
if ~isempty(singleFile)
    if ~isfile(singleFile)
        error('指定文件不存在: %s', singleFile);
    end
    csvFiles = {singleFile};
    return;
end
d = dir(fullfile(dataRoot, '**', 'session_data.csv'));
csvFiles = arrayfun(@(x) fullfile(x.folder, x.name), d, 'UniformOutput', false);
if isempty(csvFiles)
    error('在 %s 下未找到 session_data.csv', dataRoot);
end
end

function out = evaluateOneFile(mdl, cfg, csvPath, outDir, writePlot)
tbl = readtable(csvPath);
q = getCols(tbl, 'Act_q');
dq = getCols(tbl, 'Act_qd');
iMeas = getCols(tbl, 'Act_I');

n = min([size(q,1), size(dq,1), size(iMeas,1)]);
if n < 5
    error('样本太短: %s', csvPath);
end
q = q(1:n,:);
dq = dq(1:n,:);
iMeas = iMeas(1:n,:);
t = (0:n-1)' * cfg.sampleTime;

ddq = [zeros(1,6); diff(dq,1,1)/max(cfg.sampleTime,eps)];
iRefZero = zeros(n,6);

in = Simulink.SimulationInput(mdl);
in = in.setVariable('q_cmd_ts', timeseries(q, t));
in = in.setVariable('q_ref_ts', timeseries(q, t));
in = in.setVariable('dq_ref_ts', timeseries(dq, t));
in = in.setVariable('tau_csv_ref_ts', timeseries(iRefZero, t));
in = in.setVariable('tau_from_i_ref_ts', timeseries(iRefZero, t));
in = in.setModelParameter('StopTime', num2str(t(end)));
simOut = sim(in);

y = simOut.yout;
if isa(y, 'Simulink.SimulationData.Dataset')
    iEst = y.getElement(4).Values.Data;
else
    iEst = y.signals(4).values;
end

m = min(size(iEst,1), size(iMeas,1));
iEst = iEst(1:m,:);
iMeas = iMeas(1:m,:);
tUse = t(1:m);
err = iEst - iMeas;

rmse = sqrt(mean(err.^2,1));
mae = mean(abs(err),1);
mx = max(abs(err),[],1);

rel = strrep(csvPath, [fileparts(fileparts(cfg.baseDir)) filesep], '');
safeName = regexprep(rel, '[\\/:*?"<>| ]', '_');

if writePlot
    fig = figure('Visible','off','Color','w','Position',[100 80 1400 900]);
    tl = tiledlayout(fig, 3, 2, 'Padding','compact', 'TileSpacing','compact');
    for j = 1:6
        nexttile(tl, j);
        plot(tUse, iMeas(:,j), 'k-', 'LineWidth', 1.0); hold on;
        plot(tUse, iEst(:,j), 'r-', 'LineWidth', 1.0);
        plot(tUse, err(:,j), 'b--', 'LineWidth', 0.8);
        grid on;
        title(sprintf('J%d  RMSE=%.4f', j, rmse(j)));
        xlabel('t (s)');
        ylabel('current (A)');
        legend({'I_{meas}','I_{est}','err'}, 'Location','best');
    end
    exportgraphics(fig, fullfile(outDir, [safeName '_current_compare.png']));
    close(fig);
end

row = struct();
row.file = string(csvPath);
for j = 1:6
    row.(sprintf('rmse_j%d', j)) = rmse(j);
    row.(sprintf('mae_j%d', j)) = mae(j);
    row.(sprintf('maxabs_j%d', j)) = mx(j);
end
row.rmse_mean = mean(rmse);
row.mae_mean = mean(mae);
row.maxabs_max = max(mx);
row.samples = m;
row.ddq_rms = sqrt(mean(ddq(1:m,:).^2, 'all'));

out = struct();
out.metricsRow = row;
end

function mat = getCols(tbl, prefix)
mat = zeros(height(tbl),6);
for j = 1:6
    name = [prefix num2str(j-1)];
    if ~ismember(name, tbl.Properties.VariableNames)
        error('缺少列: %s', name);
    end
    mat(:,j) = tbl.(name);
end
end

function overall = computeOverall(perFileTbl)
overall = table();
joint = (1:6)';
overall.joint = joint;
for f = {'rmse','mae','maxabs'}
    fname = f{1};
    vals = zeros(6,1);
    p95 = zeros(6,1);
    for j = 1:6
        col = sprintf('%s_j%d', fname, j);
        arr = perFileTbl.(col);
        vals(j) = mean(arr, 'omitnan');
        p95(j) = prctile(arr, 95);
    end
    overall.([fname '_mean']) = vals;
    overall.([fname '_p95']) = p95;
end
end

function writeReport(path, perFileTbl, overall, worstCases, failed, dataRoot)
fid = fopen(path, 'w');
if fid < 0
    error('无法写报告: %s', path);
end
cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>

fprintf(fid, '# Current Estimation Batch Report\n\n');
fprintf(fid, '- Data root: `%s`\n', dataRoot);
fprintf(fid, '- Success files: %d\n', height(perFileTbl));
fprintf(fid, '- Failed files: %d\n\n', size(failed,1));

fprintf(fid, '## Overall Per-Joint Metrics\n\n');
fprintf(fid, '| Joint | RMSE mean | RMSE p95 | MAE mean | MAE p95 | MaxAbs mean | MaxAbs p95 |\n');
fprintf(fid, '|---|---:|---:|---:|---:|---:|---:|\n');
for i = 1:height(overall)
    fprintf(fid, '| J%d | %.6f | %.6f | %.6f | %.6f | %.6f | %.6f |\n', ...
        overall.joint(i), overall.rmse_mean(i), overall.rmse_p95(i), ...
        overall.mae_mean(i), overall.mae_p95(i), ...
        overall.maxabs_mean(i), overall.maxabs_p95(i));
end
fprintf(fid, '\n');

fprintf(fid, '## Worst Cases (Top 10 by rmse_mean)\n\n');
fprintf(fid, '| File | rmse_mean | maxabs_max | samples |\n');
fprintf(fid, '|---|---:|---:|---:|\n');
for i = 1:height(worstCases)
    fprintf(fid, '| `%s` | %.6f | %.6f | %d |\n', ...
        char(worstCases.file(i)), worstCases.rmse_mean(i), worstCases.maxabs_max(i), worstCases.samples(i));
end
fprintf(fid, '\n');

if ~isempty(failed)
    fprintf(fid, '## Failed Files\n\n');
    for i = 1:size(failed,1)
        fprintf(fid, '- `%s`: %s\n', failed{i,1}, failed{i,2});
    end
    fprintf(fid, '\n');
end
end
