function app = real_csv_fit_gui(autoCloseSec)
% REAL_CSV_FIT_GUI 真机 CSV 轨迹拟合控制界面。

clc;
if nargin < 1
    autoCloseSec = inf;
end

baseDir = fileparts(mfilename('fullpath'));
addpath(baseDir);

realCsvFitCleanupOldInstances();
cfg = realCsvFitDefaultConfig(baseDir);

state = initState();

app = createUi();
updateButtons();

if isfinite(autoCloseSec)
    state.autoTimer = timer('ExecutionMode','singleShot','StartDelay',autoCloseSec, ...
        'TimerFcn',@(~,~)localCloseFig(app.fig));
    start(state.autoTimer);
end

try
    [state.robot, state.meta] = realCsvFitLoadRobot(cfg);
    state.qCurrent = zeros(1,6);
    state.qRender = state.qCurrent;
    renderCurrent(false);
    if ~isempty(cfg.defaultCsvPath)
        state.csvPath = cfg.defaultCsvPath;
        app.txtCsv.String = ['CSV：' realCsvFitCompactPath(state.csvPath)];
    end
    setMsg('URDF加载成功。先连接真机，再读取CSV。');
catch ME
    setMsg(['URDF加载失败：' ME.message]);
end

    function stateOut = initState()
        stateOut = struct();
        stateOut.robot = [];
        stateOut.meta = struct();
        stateOut.qCurrent = zeros(1,6);
        stateOut.qRender = zeros(1,6);
        stateOut.streamOk = false;
        stateOut.lastQTs = NaN;
        stateOut.readClient = [];
        stateOut.rxBuffer = zeros(0,1,'uint8');
        stateOut.expectedPacketLen = 0;
        stateOut.resyncCount = 0;
        stateOut.bytesInTotal = 0;
        stateOut.loopTimer = [];
        stateOut.previewTimer = [];
        stateOut.autoTimer = [];
        stateOut.previewStartTic = [];
        stateOut.previewMode = false;
        stateOut.csvPath = '';
        stateOut.data = [];
        stateOut.fit = [];
        stateOut.previewed = false;
        stateOut.startCommanded = false;
        stateOut.isClosing = false;
        stateOut.renderBusy = false;
        stateOut.debugTs = tic;
    end

    function ui = createUi()
        ui.fig = figure('Name','UR10 真机CSV轨迹拟合控制','NumberTitle','off', ...
            'Position',[70 70 1450 860], 'Color','w', 'MenuBar','none', ...
            'ToolBar','none', 'CloseRequestFcn',@onClose);
        ui.ax = axes('Parent',ui.fig,'Units','pixels','Position',[20 170 930 650]);
        axis(ui.ax,'equal');
        grid(ui.ax,'on');
        view(ui.ax,135,20);
        disableDefaultInteractivity(ui.ax);
        ui.ax.Interactions = [];
        ui.ax.PickableParts = 'none';
        ui.ax.HitTest = 'off';
        ui.ax.UIContextMenu = [];
        try
            if ~isempty(ui.ax.Toolbar)
                ui.ax.Toolbar.Visible = 'off';
            end
        catch
        end
        realCsvFitDisableAxesInteractions(ui.ax);

        ui.txtMode = textUi([980 805 430 24], '状态：未连接');
        ui.txtStream = textUi([980 775 430 24], '流状态：N/A');
        ui.txtCsv = textUi([980 745 430 24], 'CSV：未选择');
        ui.txtFit = textUi([980 700 430 38], '拟合：未执行');
        ui.txtExec = textUi([980 670 430 24], '执行：未就绪');
        ui.txtMsg = textUi([20 130 1390 28], '消息：准备就绪');

        ui.btnConnect = buttonUi([980 620 100 32], '连接', @onConnect);
        ui.btnDisconnect = buttonUi([1090 620 100 32], '断开', @onDisconnect);
        ui.btnPick = buttonUi([1200 620 120 32], '选择CSV', @onPickCsv);
        ui.btnLoad = buttonUi([1330 620 90 32], '读取拟合', @onLoadAndFit);

        ui.btnMoveStart = buttonUi([980 575 140 34], '移动到起点', @onMoveToStart);
        ui.btnPreview = buttonUi([1130 575 120 34], '预演', @onPreview);
        ui.btnExecute = buttonUi([1260 575 160 34], '执行轨迹', @onExecuteTrajectory);
        ui.btnStop = buttonUi([980 530 440 34], '暂停/停止', @onStop);
        ui.btnStop.BackgroundColor = [0.9 0.25 0.25];
        ui.btnStop.ForegroundColor = 'w';

        textUi([980 490 120 22], '时间放慢倍数');
        ui.edtTimeScale = uicontrol(ui.fig,'Style','edit','Position',[1105 492 70 24], ...
            'String',sprintf('%.1f',cfg.timeScale),'Callback',@onTimeScaleChanged);
        textUi([1190 490 110 22], 'servoj最小t(s)');
        ui.edtServoT = uicontrol(ui.fig,'Style','edit','Position',[1305 492 70 24], ...
            'String',sprintf('%.2f',cfg.servojMinT),'Callback',@onServoTChanged);

        ui.tbl = uitable(ui.fig,'Position',[980 225 440 250], ...
            'ColumnName',{'Joint','q_RMSE(rad)','dq_RMSE(rad/s)','poly_order'}, ...
            'ColumnEditable',[false false false false], 'Data',cell(6,4));
        ui.txtQ = textUi([980 180 440 34], '当前关节角(rad)：N/A');

        function h = textUi(pos, str)
            h = uicontrol(ui.fig,'Style','text','Position',pos, ...
                'HorizontalAlignment','left','BackgroundColor','w','String',str);
        end

        function h = buttonUi(pos, str, cb)
            h = uicontrol(ui.fig,'Style','pushbutton','Position',pos, ...
                'String',str,'Callback',cb);
        end
    end

    function onConnect(~,~)
        if state.isClosing || isempty(state.robot)
            setMsg('模型未就绪，无法连接。');
            return;
        end
        try
            if ~isempty(state.readClient)
                clear state.readClient
            end
            state.readClient = tcpclient(cfg.robotIp, cfg.readPort, 'Timeout', 2);
            flush(state.readClient);
            state.rxBuffer = zeros(0,1,'uint8');
            state.expectedPacketLen = 0;
            state.streamOk = false;
            state.previewed = false;
            state.startCommanded = false;
            if isempty(state.loopTimer) || ~isvalid(state.loopTimer)
                state.loopTimer = timer('ExecutionMode','fixedSpacing','Period',cfg.samplePeriod, ...
                    'BusyMode','drop','TimerFcn',@onLoop,'Tag','UR10_REAL_CSV_FIT_LOOP');
            end
            if strcmp(state.loopTimer.Running,'off')
                start(state.loopTimer);
            end
            setMode('实时同步');
            updateButtons();
            setMsg(sprintf('已连接真机实时流 %s:%d。', cfg.robotIp, cfg.readPort));
        catch ME
            setMsg(['连接失败：' ME.message]);
        end
    end

    function onDisconnect(~,~)
        stopPreview();
        stopLoop();
        if ~isempty(state.readClient)
            clear state.readClient
            state.readClient = [];
        end
        state.streamOk = false;
        state.previewed = false;
        state.startCommanded = false;
        setMode('未连接');
        setStream('N/A');
        updateButtons();
        setMsg('已断开连接。');
    end

    function onPickCsv(~,~)
        startDir = fileparts(state.csvPath);
        if isempty(startDir) || ~isfolder(startDir)
            startDir = fileparts(cfg.projectDir);
        end
        [fileName, pathName] = uigetfile({'*.csv','CSV 文件 (*.csv)'}, ...
            '选择session_data.csv或同格式CSV', startDir);
        if isequal(fileName,0)
            return;
        end
        state.csvPath = fullfile(pathName, fileName);
        state.data = [];
        state.fit = [];
        state.previewed = false;
        state.startCommanded = false;
        state.qRender = state.qCurrent;
        app.txtCsv.String = ['CSV：' realCsvFitCompactPath(state.csvPath)];
        app.txtFit.String = '拟合：未执行';
        app.txtExec.String = '执行：未就绪';
        app.tbl.Data = cell(6,4);
        updateButtons();
        renderCurrent(false);
        setMsg(['已选择CSV：' state.csvPath]);
    end

    function onLoadAndFit(~,~)
        try
            if isempty(state.csvPath)
                error('请先选择CSV。');
            end
            data = realCsvFitReadCsv(state.csvPath, cfg.defaultSampleTime);
            fit = realCsvFitFitTrajectory(data, cfg.polyOrder);
            [ok,msg] = realCsvFitCheckLimits(fit.qFit, cfg.jointLimitsRad);
            if ~ok
                error(msg);
            end
            state.data = data;
            state.fit = fit;
            state.qRender = fit.qFit(1,:);
            state.previewed = false;
            state.startCommanded = false;
            app.tbl.Data = realCsvFitBuildTable(fit);
            app.txtFit.String = sprintf('拟合：%d点, 时长 %.3fs, q平均RMSE %.5f, dq平均RMSE %.5f', ...
                data.n, data.t(end), mean(fit.qRmse), mean(fit.dqRmse));
            app.txtExec.String = '执行：已读取，下一步移动到起点并预演';
            updateButtons();
            renderCurrent(true);
            setMsg(sprintf('读取并拟合完成：%s', state.csvPath));
        catch ME
            state.data = [];
            state.fit = [];
            state.previewed = false;
            state.startCommanded = false;
            updateButtons();
            setMsg(['读取/拟合失败：' ME.message]);
        end
    end

    function onMoveToStart(~,~)
        if isempty(state.fit)
            setMsg('请先读取并拟合CSV。');
            return;
        end
        if ~state.streamOk
            setMsg('实时流未就绪，禁止移动到起点。');
            return;
        end
        try
            stopPreview();
            qStart = state.fit.qFit(1,:);
            [ok,msg] = realCsvFitCheckLimits(qStart, cfg.jointLimitsRad);
            if ~ok
                setMsg(msg);
                return;
            end
            realCsvFitSendMoveJ(cfg.robotIp, cfg.cmdPort, qStart, cfg.movejA, cfg.movejV, cfg.movejT, cfg.movejR);
            state.qRender = qStart;
            state.startCommanded = true;
            state.previewed = false;
            setMode('移动到起点中');
            app.txtExec.String = sprintf('执行：已发送起点movej，目标距离 %.3f rad', norm(state.qCurrent - qStart));
            updateButtons();
            renderCurrent(true);
            setMsg('已发送低速movej到CSV起点。等真机到位后点击预演。');
        catch ME
            setMsg(['移动到起点失败：' ME.message]);
        end
    end

    function onPreview(~,~)
        if isempty(state.fit)
            setMsg('请先读取并拟合CSV。');
            return;
        end
        try
            stopPreview();
            state.previewStartTic = tic;
            state.previewMode = true;
            state.previewed = false;
            if isempty(state.previewTimer) || ~isvalid(state.previewTimer)
                state.previewTimer = timer('ExecutionMode','fixedSpacing','Period',cfg.previewPeriod, ...
                    'BusyMode','drop','TimerFcn',@onPreviewStep,'Tag','UR10_REAL_CSV_FIT_PREVIEW');
            end
            start(state.previewTimer);
            setMode('预演中');
            app.txtExec.String = '执行：预演中，仅MATLAB动画，不发送真机命令';
            updateButtons();
            setMsg('开始预演拟合轨迹。预演不控制真实机械臂。');
        catch ME
            state.previewMode = false;
            updateButtons();
            setMsg(['预演失败：' ME.message]);
        end
    end

    function onPreviewStep(~,~)
        if state.isClosing || isempty(state.fit) || ~state.previewMode
            return;
        end
        try
            elapsed = toc(state.previewStartTic);
            tEnd = state.fit.t(end);
            k = find(state.fit.t <= elapsed, 1, 'last');
            if isempty(k)
                k = 1;
            end
            if elapsed >= tEnd
                k = size(state.fit.qFit,1);
            end
            state.qRender = state.fit.qFit(k,:);
            renderCurrent(true);
            app.txtExec.String = sprintf('执行：预演 %d/%d, t=%.2fs', k, size(state.fit.qFit,1), state.fit.t(k));
            if elapsed >= tEnd
                stopPreview();
                state.previewed = true;
                setMode('预演完成');
                app.txtExec.String = '执行：预演完成，可在真机位于起点后执行轨迹';
                updateButtons();
                setMsg('预演完成。确认轨迹无异常后，可执行真机轨迹。');
            end
        catch ME
            stopPreview();
            updateButtons();
            setMsg(['预演中断：' ME.message]);
        end
    end

    function onExecuteTrajectory(~,~)
        if isempty(state.fit)
            setMsg('请先读取并拟合CSV。');
            return;
        end
        if ~state.streamOk
            setMsg('实时流超时，禁止执行。');
            return;
        end
        if ~state.previewed
            setMsg('请先完成预演。');
            return;
        end
        qStart = state.fit.qFit(1,:);
        distStart = norm(state.qCurrent - qStart);
        if distStart > cfg.startNearTolRad
            setMsg(sprintf('真机未到CSV起点，当前距离 %.3f rad，大于阈值 %.3f rad。', distStart, cfg.startNearTolRad));
            return;
        end
        try
            stopPreview();
            [qExec, dtExec] = realCsvFitBuildExecutionTrajectory(state.fit.t, state.fit.qFit, cfg);
            [ok,msg] = realCsvFitCheckLimits(qExec, cfg.jointLimitsRad);
            if ~ok
                setMsg(msg);
                return;
            end
            script = realCsvFitBuildServoJScript(qExec, dtExec, cfg);
            realCsvFitSendScript(cfg.robotIp, cfg.cmdPort, script);
            state.previewed = false;
            state.startCommanded = false;
            setMode('真机执行中');
            app.txtExec.String = sprintf('执行：已发送servoj轨迹，点数 %d，预计 %.1fs', size(qExec,1), sum(dtExec));
            updateButtons();
            setMsg('已发送低速servoj轨迹。需要中断时点击“暂停/停止”。');
        catch ME
            setMsg(['执行轨迹失败：' ME.message]);
        end
    end

    function onStop(~,~)
        try
            stopPreview();
            realCsvFitSendStopJ(cfg.robotIp, cfg.cmdPort, cfg.stopDecel);
            state.previewed = false;
            state.startCommanded = false;
            if ~strcmp(getMode(),'未连接')
                setMode('已停止');
            end
            app.txtExec.String = '执行：已发送stopj，继续前请重新移动到起点并预演';
            updateButtons();
            setMsg('已发送stopj，轨迹状态已锁定。');
        catch ME
            setMsg(['暂停/停止失败：' ME.message]);
        end
    end

    function onTimeScaleChanged(src,~)
        v = str2double(src.String);
        if ~isfinite(v) || v < 1
            v = cfg.timeScale;
        end
        cfg.timeScale = v;
        src.String = sprintf('%.1f', cfg.timeScale);
        setMsg(sprintf('时间放慢倍数已设置为 %.1f。', cfg.timeScale));
    end

    function onServoTChanged(src,~)
        v = str2double(src.String);
        if ~isfinite(v) || v < 0.02
            v = cfg.servojMinT;
        end
        cfg.servojMinT = v;
        src.String = sprintf('%.2f', cfg.servojMinT);
        setMsg(sprintf('servoj最小t已设置为 %.2fs。', cfg.servojMinT));
    end

    function onLoop(~,~)
        if state.isClosing || isempty(state.readClient) || ~isgraphics(app.fig)
            return;
        end
        try
            [q,state.rxBuffer,newPk,state.expectedPacketLen,lost,bytesNow] = ...
                realCsvFitReadLatestUrQActual(state.readClient,state.rxBuffer,state.expectedPacketLen);
            state.bytesInTotal = state.bytesInTotal + bytesNow;
            if lost
                state.resyncCount = state.resyncCount + 1;
            end
            if ~isempty(q)
                state.qCurrent = q;
                state.lastQTs = tic;
                state.streamOk = true;
                if ~state.previewMode
                    state.qRender = q;
                end
            end
            age = getStreamAge();
            if age > cfg.streamStaleSec
                state.streamOk = false;
                setStream(sprintf('超时 age=%.2fs', age));
            else
                setStream(sprintf('正常 age=%.3f 包长=%d 新包=%d 字节=%d 重同步=%d', ...
                    age,state.expectedPacketLen,newPk,state.bytesInTotal,state.resyncCount));
            end
            app.txtQ.String = sprintf('当前关节角(rad)：[%.3f %.3f %.3f %.3f %.3f %.3f]', state.qCurrent);
            updateButtons();
            if ~state.previewMode
                renderCurrent(false);
            end
            if toc(state.debugTs) > 2
                fprintf('[REAL_CSV_FIT] stream=%d q=[%.3f %.3f %.3f %.3f %.3f %.3f]\n', ...
                    state.streamOk,state.qCurrent);
                state.debugTs = tic;
            end
        catch ME
            state.streamOk = false;
            setStream(['错误：' ME.message]);
            updateButtons();
            fprintf(2,'[REAL_CSV_FIT][WARN] loop failed: %s\n', ME.message);
        end
    end

    function age = getStreamAge()
        age = inf;
        if ~isnan(state.lastQTs)
            age = toc(state.lastQTs);
        end
    end

    function renderCurrent(showPath)
        if state.isClosing || state.renderBusy || isempty(state.robot) || ~isgraphics(app.ax)
            return;
        end
        state.renderBusy = true;
        try
            realCsvFitRenderRobot(app.ax, state.robot, state.qRender, showPath, getPathForRender(), cfg);
        catch ME
            fprintf(2,'[REAL_CSV_FIT][WARN] render dispatch failed: %s\n', ME.message);
        end
        setRenderIdle();
    end

    function qPath = getPathForRender()
        qPath = [];
        if ~isempty(state.fit)
            qPath = state.fit.qFit;
        end
    end

    function setRenderIdle()
        state.renderBusy = false;
    end

    function updateButtons()
        hasFit = ~isempty(state.fit);
        app.btnMoveStart.Enable = onOff(hasFit && state.streamOk);
        app.btnPreview.Enable = onOff(hasFit && state.startCommanded && ~state.previewMode);
        app.btnExecute.Enable = onOff(hasFit && state.streamOk && state.previewed && ~state.previewMode);
    end

    function setMode(m)
        if isgraphics(app.txtMode)
            app.txtMode.String = ['状态：' m];
        end
    end

    function m = getMode()
        m = '';
        if isgraphics(app.txtMode)
            s = app.txtMode.String;
            p = strfind(s,'：');
            if ~isempty(p)
                m = s(p(1)+1:end);
            else
                m = s;
            end
        end
    end

    function setStream(s)
        if isgraphics(app.txtStream)
            app.txtStream.String = ['流状态：' s];
        end
    end

    function setMsg(s)
        if isgraphics(app.txtMsg)
            app.txtMsg.String = ['消息：' s];
        end
        fprintf('[REAL_CSV_FIT] %s\n', s);
    end

    function stopLoop()
        if ~isempty(state.loopTimer) && isvalid(state.loopTimer) && strcmp(state.loopTimer.Running,'on')
            stop(state.loopTimer);
        end
    end

    function stopPreview()
        state.previewMode = false;
        state.previewStartTic = [];
        if ~isempty(state.previewTimer) && isvalid(state.previewTimer) && strcmp(state.previewTimer.Running,'on')
            stop(state.previewTimer);
        end
    end

    function onClose(~,~)
        state.isClosing = true;
        stopPreview();
        stopLoop();
        deleteTimer('previewTimer');
        deleteTimer('loopTimer');
        deleteTimer('autoTimer');
        if ~isempty(state.readClient)
            clear state.readClient
            state.readClient = [];
        end
        if isgraphics(app.fig)
            delete(app.fig);
        end
    end

    function deleteTimer(fieldName)
        t = state.(fieldName);
        if ~isempty(t) && isvalid(t)
            if strcmp(t.Running,'on')
                stop(t);
            end
            delete(t);
            state.(fieldName) = [];
        end
    end
end

function out = onOff(tf)
if tf
    out = 'on';
else
    out = 'off';
end
end

function localCloseFig(fig)
if ~isempty(fig) && isgraphics(fig)
    close(fig);
end
end
