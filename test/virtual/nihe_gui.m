function app = nihe_gui()
% 拟合回放GUI:
% 1) 选择包含 session_data.csv 的文件夹
% 2) 自动解析关节角/速度/电流
% 3) 拟合关节角/速度随时间变化
% 4) 点击播放执行运动动画

baseDir = fileparts(mfilename('fullpath'));
addpath(baseDir);
addpath(fullfile(baseDir, 'utils'));

cfg = init_virtual_ur10(baseDir);
[robot, meta] = build_ur10_virtual_robot(cfg);

state = struct();
state.cfg = cfg;
state.robot = robot;
state.meta = meta;
state.folder = '';
state.csvPath = '';
state.data = [];
state.fit = [];
state.timer = [];
state.playIdx = 1;
state.playStride = 1;
state.isPlaying = false;
state.playStartTic = [];

app.fig = figure( ...
    'Name', 'nihe_gui - 数据拟合与动画回放', ...
    'NumberTitle', 'off', ...
    'Color', 'w', ...
    'Position', [80 80 1450 860], ...
    'CloseRequestFcn', @onClose);

app.ax = axes('Parent', app.fig, 'Units', 'pixels', 'Position', [20 160 920 670]);
axis(app.ax, 'equal');
grid(app.ax, 'on');
view(app.ax, cfg.guiView(1), cfg.guiView(2));
disableDefaultInteractivity(app.ax);
app.ax.Interactions = [];
app.ax.PickableParts = 'none';
app.ax.HitTest = 'off';
app.ax.Toolbar.Visible = 'off';
app.ax.UIContextMenu = [];
localDisableAxesTextInteractions(app.ax);

app.txtFolder = uicontrol(app.fig, 'Style', 'text', 'Position', [960 790 460 24], ...
    'HorizontalAlignment', 'left', 'BackgroundColor', 'w', 'String', '数据目录: 未选择');
app.btnFolder = uicontrol(app.fig, 'Style', 'pushbutton', 'Position', [960 755 150 30], ...
    'String', '选择文件夹', 'Callback', @onPickFolder);
app.btnLoad = uicontrol(app.fig, 'Style', 'pushbutton', 'Position', [1120 755 150 30], ...
    'String', '读取并拟合', 'Callback', @onLoadAndFit);
app.btnPlay = uicontrol(app.fig, 'Style', 'pushbutton', 'Position', [1280 755 140 30], ...
    'String', '播放', 'Enable', 'off', 'Callback', @onPlay);
app.btnStop = uicontrol(app.fig, 'Style', 'pushbutton', 'Position', [1280 720 140 30], ...
    'String', '停止', 'Enable', 'off', 'Callback', @onStop);

app.txtInfo = uicontrol(app.fig, 'Style', 'text', 'Position', [960 680 460 32], ...
    'HorizontalAlignment', 'left', 'BackgroundColor', 'w', 'String', '数据状态: 未加载');
app.txtFit = uicontrol(app.fig, 'Style', 'text', 'Position', [960 620 460 54], ...
    'HorizontalAlignment', 'left', 'BackgroundColor', 'w', 'String', '拟合状态: 未执行');
app.txtPlay = uicontrol(app.fig, 'Style', 'text', 'Position', [960 580 460 24], ...
    'HorizontalAlignment', 'left', 'BackgroundColor', 'w', 'String', '播放状态: 未播放');

app.tbl = uitable(app.fig, 'Position', [960 300 460 270], ...
    'ColumnName', {'Joint', 'q_RMSE(rad)', 'dq_RMSE(rad/s)', 'poly_order'}, ...
    'Data', cell(6,4));

app.txtLog = uicontrol(app.fig, 'Style', 'text', 'Position', [20 120 1400 28], ...
    'HorizontalAlignment', 'left', 'BackgroundColor', 'w', 'String', '日志: 就绪');

renderRobot(cfg.homeQ, false);

    function onPickFolder(~,~)
        d = uigetdir('E:\UR10_control\test\data', '选择包含 session_data.csv 的目录');
        if isequal(d,0)
            return;
        end
        state.folder = d;
        state.csvPath = fullfile(d, 'session_data.csv');
        app.txtFolder.String = ['数据目录: ' state.folder];
        logMsg(['已选择目录: ' state.folder]);
    end

    function onLoadAndFit(~,~)
        try
            if isempty(state.folder)
                error('请先选择目录。');
            end
            if ~isfile(state.csvPath)
                error('未找到文件: %s', state.csvPath);
            end

            d = loadCsv(state.csvPath, cfg.sampleTime);
            f = fitCurrentModel(d);
            state.data = d;
            state.fit = f;
            state.playIdx = 1;

            app.txtInfo.String = sprintf('数据状态: 已加载 %d 点, 时长 %.2f s', d.n, d.t(end));
            app.txtFit.String = sprintf('拟合状态: 完成, q平均RMSE=%.4f, dq平均RMSE=%.4f', ...
                mean(f.qRmse), mean(f.dqRmse));
            app.tbl.Data = buildFitTable(f);
            app.btnPlay.Enable = 'on';
            logMsg(sprintf('读取并拟合完成: %s', state.csvPath));
        catch ME
            logMsg(['读取/拟合失败: ' ME.message]);
        end
    end

    function onPlay(~,~)
        if isempty(state.data) || isempty(state.fit)
            logMsg('请先读取并拟合数据。');
            return;
        end
        if state.isPlaying
            return;
        end
        state.isPlaying = true;
        state.playStartTic = tic;
        app.btnPlay.Enable = 'off';
        app.btnStop.Enable = 'on';
        app.txtPlay.String = '播放状态: 播放中';

        if isempty(state.timer) || ~isvalid(state.timer)
            state.timer = timer('ExecutionMode','fixedSpacing', ...
                'Period', max(min(cfg.sampleTime,0.01),0.005), ...
                'BusyMode','drop', ...
                'TimerFcn', @onTimerStep);
        end
        start(state.timer);
    end

    function onStop(~,~)
        stopPlayback();
    end

    function onTimerStep(~,~)
        if ~state.isPlaying || isempty(state.data)
            return;
        end
        elapsed = toc(state.playStartTic);
        k = floor(elapsed / max(state.cfg.sampleTime, eps)) + 1;
        if k < 1
            k = 1;
        end
        if k > state.data.n
            stopPlayback();
            return;
        end
        qk = state.fit.qFit(k,:);
        renderRobot(qk, true);
        app.txtPlay.String = sprintf('播放状态: %d / %d (t=%.2fs)', k, state.data.n, state.data.t(k));
        state.playIdx = k;
    end

    function stopPlayback()
        state.isPlaying = false;
        state.playStartTic = [];
        if ~isempty(state.timer) && isvalid(state.timer) && strcmp(state.timer.Running,'on')
            stop(state.timer);
        end
        app.btnPlay.Enable = 'on';
        app.btnStop.Enable = 'off';
        app.txtPlay.String = '播放状态: 已停止';
    end

    function renderRobot(qrow, fastUpdate)
        if nargin < 2
            fastUpdate = true;
        end
        if ~isgraphics(app.ax)
            return;
        end
        try
            show(state.robot, qrow, 'Parent', app.ax, ...
                'Visuals', 'on', 'Collisions', 'off', 'Frames', 'off', ...
                'PreservePlot', false, 'FastUpdate', fastUpdate);
            axis(app.ax, 'equal');
            grid(app.ax, 'on');
            drawnow limitrate nocallbacks;
        catch ME
            fprintf(2, '[NIHE][WARN] render failed: %s\n', ME.message);
        end
    end

    function logMsg(s)
        app.txtLog.String = ['日志: ' s];
        fprintf('[NIHE] %s\n', s);
    end

    function onClose(~,~)
        stopPlayback();
        if ~isempty(state.timer) && isvalid(state.timer)
            delete(state.timer);
        end
        if isgraphics(app.fig)
            delete(app.fig);
        end
    end
end

function d = loadCsv(csvPath, sampleTime)
tbl = readtable(csvPath);
q = getCols(tbl, 'Act_q');
dq = getCols(tbl, 'Act_qd');

n = min([size(q,1), size(dq,1)]);
if n < 5
    error('数据点太少，无法拟合。');
end
q = q(1:n,:);
dq = dq(1:n,:);
t = (0:n-1)' * sampleTime;

d = struct();
d.q = q;
d.dq = dq;
d.t = t;
d.n = n;
end

function f = fitCurrentModel(d)
% 每个关节独立拟合: q(t), dq(t) 采用三次多项式
order = 3;
qCoef = zeros(6, order+1);
dqCoef = zeros(6, order+1);
qFit = zeros(d.n,6);
dqFit = zeros(d.n,6);
qRmse = zeros(6,1);
dqRmse = zeros(6,1);
for j = 1:6
    pQ = polyfit(d.t, d.q(:,j), order);
    pDQ = polyfit(d.t, d.dq(:,j), order);
    qh = polyval(pQ, d.t);
    dqh = polyval(pDQ, d.t);
    qCoef(j,:) = pQ;
    dqCoef(j,:) = pDQ;
    qFit(:,j) = qh;
    dqFit(:,j) = dqh;
    qRmse(j) = sqrt(mean((qh - d.q(:,j)).^2));
    dqRmse(j) = sqrt(mean((dqh - d.dq(:,j)).^2));
end

f = struct();
f.qCoef = qCoef;
f.dqCoef = dqCoef;
f.qFit = qFit;
f.dqFit = dqFit;
f.qRmse = qRmse;
f.dqRmse = dqRmse;
f.order = order;
end

function out = buildFitTable(f)
out = cell(6,4);
for j = 1:6
    out{j,1} = sprintf('J%d', j);
    out{j,2} = f.qRmse(j);
    out{j,3} = f.dqRmse(j);
    out{j,4} = f.order;
end
end

function mat = getCols(tbl, prefix)
mat = zeros(height(tbl),6);
for j = 1:6
    col = [prefix num2str(j-1)];
    if ~ismember(col, tbl.Properties.VariableNames)
        error('CSV缺少列: %s', col);
    end
    mat(:,j) = tbl.(col);
end
end

function localDisableAxesTextInteractions(ax)
try
    ax.Interactions = [];
catch
end
try
    ax.Title.Interactions = [];
catch
end
try
    ax.XLabel.Interactions = [];
catch
end
try
    ax.YLabel.Interactions = [];
catch
end
try
    ax.ZLabel.Interactions = [];
catch
end
end
