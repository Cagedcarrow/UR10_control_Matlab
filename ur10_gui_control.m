function ur10_gui_control(autoCloseSec)
% UR10 GUI（无碰撞检测版）
% 目标：稳定实时同步 + 滑轨设目标 + 点击开始运动可执行

clc;
if nargin < 1
    autoCloseSec = inf;
end

% 清理旧实例
oldTimers = timerfindall('Tag','UR10_GUI_LOOP');
for i = 1:numel(oldTimers)
    try, stop(oldTimers(i)); catch, end
    try, delete(oldTimers(i)); catch, end
end
oldNames = {'UR10 GUI Control','UR10 GUI Control (Realtime Consistency)','UR10 实时控制界面','UR10 无碰撞控制界面'};
for i = 1:numel(oldNames)
    figs = findall(groot,'Type','figure','Name',oldNames{i});
    for j = 1:numel(figs)
        try, delete(figs(j)); catch, end
    end
end

cfg.robotIp = '10.160.9.21';
cfg.readPort = 30003;
cfg.cmdPort = 30002;
cfg.xacroPath = 'E:/UR10_control/assembly/assembly.urdf.xacro';
cfg.meshRootName = 'meshes';
cfg.samplePeriod = 0.05;
cfg.streamStaleSec = 1.0;
cfg.movej_a = 0.2;
cfg.movej_v = 0.2;
cfg.movej_t = 0.0;
cfg.movej_r = 0.0;
cfg.jointNames = {'ur10_shoulder_pan','ur10_shoulder_lift','ur10_elbow','ur10_wrist_1','ur10_wrist_2','ur10_wrist_3'};
cfg.jointLimitsRad = repmat([-2*pi, 2*pi], 6, 1);

state = struct();
state.mode = '未连接';
state.robot = [];
state.qCurrent = zeros(1,6);
state.qTarget = zeros(1,6);
state.streamOk = false;
state.lastQTs = NaN;
state.readClient = [];
state.rxBuffer = zeros(0,1,'uint8');
state.expectedPacketLen = 0;
state.resyncCount = 0;
state.bytesInTotal = 0;
state.loopTimer = [];
state.debugTs = tic;
state.jointCfgIdx = 1:6;
state.validated = false;
state.targetDirty = false;
state.isClosing = false;
state.renderBusy = false;

ui.fig = figure('Name','UR10 无碰撞控制界面','NumberTitle','off','Position',[80 80 1350 800], ...
    'Color','w','MenuBar','none','ToolBar','none','CloseRequestFcn',@onClose);
ui.ax = axes('Parent',ui.fig,'Units','pixels','Position',[20 160 900 620]);
view(ui.ax,135,20); grid(ui.ax,'on'); axis(ui.ax,'equal');
disableDefaultInteractivity(ui.ax);
ui.ax.Interactions = [];
ui.ax.PickableParts = 'none';
ui.ax.HitTest = 'off';
try
    if ~isempty(ui.ax.Toolbar)
        ui.ax.Toolbar.Visible = 'off';
    end
catch
end
localDisableAxesTextInteractions(ui.ax);

ui.txtMode = uicontrol(ui.fig,'Style','text','Position',[950 760 360 22],'HorizontalAlignment','left','String','状态：未连接','BackgroundColor','w');
ui.txtStream = uicontrol(ui.fig,'Style','text','Position',[950 735 360 22],'HorizontalAlignment','left','String','流状态：N/A','BackgroundColor','w');
ui.txtMsg = uicontrol(ui.fig,'Style','text','Position',[20 120 1290 22],'HorizontalAlignment','left','String','消息：准备就绪','BackgroundColor','w');

ui.btnConnect = uicontrol(ui.fig,'Style','pushbutton','Position',[950 690 110 30],'String','连接','Callback',@onConnect);
ui.btnDisconnect = uicontrol(ui.fig,'Style','pushbutton','Position',[1070 690 110 30],'String','断开','Callback',@onDisconnect);
ui.btnSync = uicontrol(ui.fig,'Style','pushbutton','Position',[1190 690 130 30],'String','目标=当前','Callback',@onSyncTarget);
ui.btnValidate = uicontrol(ui.fig,'Style','pushbutton','Position',[950 645 180 32],'String','预演并解锁','Callback',@onValidateNoCollision);
ui.btnExec = uicontrol(ui.fig,'Style','pushbutton','Position',[1140 645 180 32],'String','开始运动','Enable','off','Callback',@onExecute);
ui.btnStop = uicontrol(ui.fig,'Style','pushbutton','Position',[950 602 370 32],'String','急停','BackgroundColor',[0.9 0.3 0.3],'Callback',@onStop);

ui.sliders = gobjects(1,6);
ui.sliderValTxt = gobjects(1,6);
for i = 1:6
    y = 520 - (i-1)*58;
    idx = i;
    uicontrol(ui.fig,'Style','text','Position',[950 y+25 170 20],'String',sprintf('第%d轴目标角(度)',i),'HorizontalAlignment','left','BackgroundColor','w');
    ui.sliders(i) = uicontrol(ui.fig,'Style','slider', ...
        'Position',[950 y 300 20], ...
        'Min',rad2deg(cfg.jointLimitsRad(i,1)), ...
        'Max',rad2deg(cfg.jointLimitsRad(i,2)), ...
        'Value',0, ...
        'Callback',@(src,evt)onSlider(idx));
    ui.sliderValTxt(i) = uicontrol(ui.fig,'Style','text','Position',[1260 y+2 70 20],'String','0.00','BackgroundColor','w');
end

if isfinite(autoCloseSec)
    tAuto = timer('ExecutionMode','singleShot','StartDelay',autoCloseSec,'TimerFcn',@(~,~)localCloseFig(ui.fig));
    start(tAuto);
end

try
    state.robot = localLoadRobotFromXacro(cfg.xacroPath, cfg.meshRootName, 'struct');
    state.jointCfgIdx = localMapJointIndices(state.robot, cfg.jointNames);
    cfgHome = homeConfiguration(state.robot);
    state.qCurrent = localCfgToRow(cfgHome, state.jointCfgIdx);
    state.qTarget = state.qCurrent;
    updateSliderValues();
    renderScene();
    setMsg('URDF 加载成功，请点击连接。');
catch ME
    setMsg(['URDF 加载失败：' ME.message]);
end

    function onConnect(~,~)
        if state.isClosing || isempty(state.robot)
            setMsg('模型未就绪，无法连接。');
            return;
        end
        try
            if ~isempty(state.readClient), clear state.readClient; end
            state.readClient = tcpclient(cfg.robotIp, cfg.readPort, 'Timeout', 2);
            flush(state.readClient);
            state.rxBuffer = zeros(0,1,'uint8');
            state.expectedPacketLen = 0;
            state.streamOk = false;
            state.validated = false;
            set(ui.btnExec,'Enable','off');
            setMode('实时同步');

            if isempty(state.loopTimer) || ~isvalid(state.loopTimer)
                state.loopTimer = timer('ExecutionMode','fixedSpacing','Period',cfg.samplePeriod, ...
                    'BusyMode','drop','TimerFcn',@onLoop,'Tag','UR10_GUI_LOOP');
            end
            if strcmp(state.loopTimer.Running,'off'), start(state.loopTimer); end
            setMsg(sprintf('已连接 %s:%d',cfg.robotIp,cfg.readPort));
        catch ME
            setMsg(['连接失败：' ME.message]);
        end
    end

    function onDisconnect(~,~)
        stopLoop();
        if ~isempty(state.readClient), clear state.readClient; state.readClient = []; end
        state.streamOk = false;
        state.validated = false;
        set(ui.btnExec,'Enable','off');
        setMode('未连接');
        setStream('N/A');
        setMsg('已断开连接。');
    end

    function onSyncTarget(~,~)
        state.qTarget = state.qCurrent;
        state.targetDirty = true;
        state.validated = false;
        set(ui.btnExec,'Enable','off');
        updateSliderValues();
        setMode('目标已设置');
        renderScene();
        setMsg('目标已同步为当前姿态。');
    end

    function onSlider(i)
        if state.isClosing, return; end
        vDeg = get(ui.sliders(i),'Value');
        state.qTarget(i) = deg2rad(vDeg);
        state.targetDirty = true;
        if isgraphics(ui.sliderValTxt(i)), set(ui.sliderValTxt(i),'String',sprintf('%.2f',vDeg)); end
        state.validated = false;
        if isgraphics(ui.btnExec), set(ui.btnExec,'Enable','off'); end
        if ~strcmp(state.mode,'未连接'), setMode('目标已设置'); end
        renderScene();
    end

    function onValidateNoCollision(~,~)
        if state.isClosing, return; end
        try
            if ~state.streamOk
                setMsg('实时流未就绪，无法解锁。');
                return;
            end
            [okLim,msgLim] = checkLimits(state.qTarget,cfg.jointLimitsRad);
            if ~okLim
                setMsg(msgLim);
                return;
            end
            state.validated = true;
            if isgraphics(ui.btnExec), set(ui.btnExec,'Enable','on'); end
            setMode('已解锁');
            setMsg('已跳过碰撞检测，已解锁开始运动。');
            renderScene();
        catch ME
            state.validated = false;
            if isgraphics(ui.btnExec), set(ui.btnExec,'Enable','off'); end
            setMsg(['解锁失败：' ME.message]);
        end
    end

    function onExecute(~,~)
        if state.isClosing, return; end
        if ~state.validated
            setMsg('未解锁，先点击“预演并解锁”。');
            return;
        end
        if ~state.streamOk
            setMsg('实时流超时，禁止执行。');
            return;
        end
        setMode('执行中');
        try
            sendMoveJ(cfg.robotIp,cfg.cmdPort,state.qTarget,cfg.movej_a,cfg.movej_v,cfg.movej_t,cfg.movej_r);
            state.validated = false;
            if isgraphics(ui.btnExec), set(ui.btnExec,'Enable','off'); end
            setMode('实时同步');
            setMsg('已发送 movej 指令。');
        catch ME
            setMode('实时同步');
            setMsg(['发送 movej 失败：' ME.message]);
        end
    end

    function onStop(~,~)
        if state.isClosing, return; end
        try
            sendStopJ(cfg.robotIp,cfg.cmdPort);
            state.validated = false;
            if isgraphics(ui.btnExec), set(ui.btnExec,'Enable','off'); end
            if ~strcmp(state.mode,'未连接'), setMode('实时同步'); end
            setMsg('已发送 stopj。');
        catch ME
            setMsg(['发送 stopj 失败：' ME.message]);
        end
    end

    function onLoop(~,~)
        if state.isClosing || isempty(state.readClient) || ~isgraphics(ui.fig), return; end
        try
            [q,state.rxBuffer,newPk,state.expectedPacketLen,lost,bytesNow] = localReadLatestUrQActual(state.readClient,state.rxBuffer,state.expectedPacketLen);
            state.bytesInTotal = state.bytesInTotal + bytesNow;
            if lost, state.resyncCount = state.resyncCount + 1; end

            if ~isempty(q)
                state.qCurrent = q;
                state.lastQTs = tic;
                state.streamOk = true;
                if ~state.targetDirty
                    state.qTarget = q;
                    updateSliderValues();
                end
            end

            age = inf;
            if ~isnan(state.lastQTs), age = toc(state.lastQTs); end
            if age > cfg.streamStaleSec
                state.streamOk = false;
                state.validated = false;
                if isgraphics(ui.btnExec), set(ui.btnExec,'Enable','off'); end
                setStream(sprintf('超时 age=%.2fs',age));
            else
                setStream(sprintf('正常 age=%.3f 包长=%d 新包=%d 字节=%d 重同步=%d', ...
                    age,state.expectedPacketLen,newPk,state.bytesInTotal,state.resyncCount));
            end

            renderScene();
            if toc(state.debugTs) > 2
                fprintf('[DEBUG] mode=%s stream=%d qCur=[%.3f %.3f %.3f %.3f %.3f %.3f] qTgt=[%.3f %.3f %.3f %.3f %.3f %.3f]\n', ...
                    state.mode,state.streamOk,state.qCurrent,state.qTarget);
                state.debugTs = tic;
            end
        catch ME
            state.streamOk = false;
            state.validated = false;
            if isgraphics(ui.btnExec), set(ui.btnExec,'Enable','off'); end
            setStream(['错误：' ME.message]);
            fprintf(2,'[WARN] loop failed: %s\n',ME.message);
        end
    end

    function renderScene()
        if state.isClosing || state.renderBusy || isempty(state.robot) || ~isgraphics(ui.ax)
            return;
        end
        state.renderBusy = true;
        c = onCleanup(@() setRenderIdle()); %#ok<NASGU>
        try
            cla(ui.ax);
            hold(ui.ax,'on');

            cfgCur = localRowToCfg(state.robot,state.qCurrent,state.jointCfgIdx);
            show(state.robot,cfgCur,'Visuals','on','Collisions','off','Frames','off', ...
                'Parent',ui.ax,'PreservePlot',false,'FastUpdate',true);
            pCur = findobj(ui.ax,'Type','Patch');
            set(pCur,'FaceAlpha',1.0,'EdgeAlpha',0.12,'HitTest','off','PickableParts','none');

            cfgTgt = localRowToCfg(state.robot,state.qTarget,state.jointCfgIdx);
            show(state.robot,cfgTgt,'Visuals','on','Collisions','off','Frames','off', ...
                'Parent',ui.ax,'PreservePlot',true);
            pAll = findobj(ui.ax,'Type','Patch');
            pGhost = setdiff(pAll,pCur);
            set(pGhost,'FaceAlpha',0.22,'EdgeAlpha',0.03,'FaceColor',[0.2 0.7 1.0],'HitTest','off','PickableParts','none');

            hold(ui.ax,'off');
            axis(ui.ax,'equal');
            grid(ui.ax,'on');
            view(ui.ax,135,20);
            localDisableAxesTextInteractions(ui.ax);
            drawnow limitrate nocallbacks;
        catch ME
            fprintf(2,'[WARN] render failed: %s\n',ME.message);
        end
    end

    function setRenderIdle()
        state.renderBusy = false;
    end

    function [ok,msg] = checkLimits(q,lim)
        bad = q < lim(:,1)' | q > lim(:,2)';
        if any(bad)
            idx = find(bad,1);
            ok = false;
            msg = sprintf('第%d轴超出软限位 [%.1f, %.1f] 度。', idx, rad2deg(lim(idx,1)), rad2deg(lim(idx,2)));
        else
            ok = true;
            msg = 'OK';
        end
    end

    function updateSliderValues()
        if state.isClosing, return; end
        for j = 1:6
            d = rad2deg(state.qTarget(j));
            if isgraphics(ui.sliders(j)), set(ui.sliders(j),'Value',d); end
            if isgraphics(ui.sliderValTxt(j)), set(ui.sliderValTxt(j),'String',sprintf('%.2f',d)); end
        end
    end

    function setMode(m)
        state.mode = m;
        if isgraphics(ui.txtMode), set(ui.txtMode,'String',['状态：' m]); end
    end

    function setStream(s)
        if isgraphics(ui.txtStream), set(ui.txtStream,'String',['流状态：' s]); end
    end

    function setMsg(s)
        if isgraphics(ui.txtMsg), set(ui.txtMsg,'String',['消息：' s]); end
        fprintf('[INFO] %s\n', s);
    end

    function stopLoop()
        if ~isempty(state.loopTimer) && isvalid(state.loopTimer)
            if strcmp(state.loopTimer.Running,'on'), stop(state.loopTimer); end
        end
    end

    function onClose(~,~)
        state.isClosing = true;
        stopLoop();
        pause(0.05);
        if ~isempty(state.loopTimer) && isvalid(state.loopTimer)
            delete(state.loopTimer);
            state.loopTimer = [];
        end
        if ~isempty(state.readClient)
            clear state.readClient
            state.readClient = [];
        end
        if isgraphics(ui.fig), delete(ui.fig); end
    end
end

function localCloseFig(fig)
if ~isempty(fig) && isgraphics(fig), close(fig); end
end

function localDisableAxesTextInteractions(ax)
try, ax.Interactions = []; catch, end
try, ax.Title.Interactions = []; catch, end
try, ax.XLabel.Interactions = []; catch, end
try, ax.YLabel.Interactions = []; catch, end
try, ax.ZLabel.Interactions = []; catch, end
end

function sendMoveJ(robotIp, cmdPort, q, a, v, t, r)
script = sprintf('movej([%.10f,%.10f,%.10f,%.10f,%.10f,%.10f],a=%.5f,v=%.5f,t=%.5f,r=%.5f)', ...
    q(1),q(2),q(3),q(4),q(5),q(6),a,v,t,r);
sendUrScript(robotIp,cmdPort,script);
end

function sendStopJ(robotIp, cmdPort)
sendUrScript(robotIp,cmdPort,'stopj(2.0)');
end

function sendUrScript(robotIp, cmdPort, script)
client = tcpclient(robotIp, cmdPort, 'Timeout', 2);
line = uint8([script newline]);
write(client, line, 'uint8');
end

function robot = localLoadRobotFromXacro(xacroPath, meshRootName, dataFormat)
if nargin < 3, dataFormat = 'struct'; end
if ~isfile(xacroPath), error('File not found: %s', xacroPath); end
assemblyDir = fileparts(xacroPath);
meshDir = fullfile(assemblyDir, meshRootName);
raw = fileread(xacroPath);
raw = regexprep(raw, '<xacro:arg[^>]*/>\s*', '');
raw = strrep(raw, '$(arg mesh_root)', meshRootName);
tmpUrdf = fullfile(tempdir, 'assembly_preprocessed_gui.urdf');
fid = fopen(tmpUrdf,'w'); if fid<0, error('Cannot write temp URDF: %s', tmpUrdf); end
c = onCleanup(@() fclose(fid)); %#ok<NASGU>
fwrite(fid,raw,'char');
robot = importrobot(tmpUrdf,'DataFormat',dataFormat,'MeshPath',meshDir);
end

function idx = localMapJointIndices(robot, desiredNames)
cfg = homeConfiguration(robot);
names = {cfg.JointName};
idx = zeros(1,numel(desiredNames));
for i = 1:numel(desiredNames)
    k = find(strcmp(names, desiredNames{i}), 1);
    if isempty(k), error('Joint name not found: %s', desiredNames{i}); end
    idx(i) = k;
end
end

function q = localCfgToRow(cfg, jointIdx)
q = zeros(1,numel(jointIdx));
for i = 1:numel(jointIdx)
    q(i) = cfg(jointIdx(i)).JointPosition;
end
end

function cfg = localRowToCfg(robot, qRow, jointIdx)
cfg = homeConfiguration(robot);
for i = 1:numel(jointIdx)
    cfg(jointIdx(i)).JointPosition = qRow(i);
end
end

function [qLatest, rxBuffer, parsedPackets, expectedPacketLen, lostSync, bytesReadNow] = localReadLatestUrQActual(tcpObj, rxBuffer, expectedPacketLen)
qLatest = [];
parsedPackets = 0;
lostSync = false;
bytesReadNow = 0;

avail = tcpObj.NumBytesAvailable;
if avail > 0
    newBytes = read(tcpObj, avail, 'uint8');
    rxBuffer = [rxBuffer; newBytes(:)]; %#ok<AGROW>
    bytesReadNow = numel(newBytes);
end

validLens = [1220,1116,1108];
while numel(rxBuffer) >= 4
    if expectedPacketLen <= 0
        [syncIdx, foundLen] = localFindSyncHeader(rxBuffer, validLens);
        if syncIdx < 1
            if numel(rxBuffer) > 3, rxBuffer = rxBuffer(end-2:end); end
            break;
        end
        if syncIdx > 1, rxBuffer = rxBuffer(syncIdx:end); end
        expectedPacketLen = foundLen;
    end

    packetLen = localBeInt32(rxBuffer(1:4));
    if packetLen ~= expectedPacketLen
        rxBuffer = rxBuffer(2:end);
        expectedPacketLen = 0;
        lostSync = true;
        continue;
    end

    if numel(rxBuffer) < packetLen
        break;
    end

    packet = rxBuffer(1:packetLen);
    rxBuffer = rxBuffer(packetLen+1:end);
    qCand = localParseQActual(packet);
    if localIsValidQ(qCand)
        parsedPackets = parsedPackets + 1;
        qLatest = qCand;
    else
        expectedPacketLen = 0;
        lostSync = true;
    end
end
end

function [idx, foundLen] = localFindSyncHeader(rxBuffer, validLens)
idx = -1; foundLen = 0;
n = numel(rxBuffer);
for k = 1:(n-3)
    L = localBeInt32(rxBuffer(k:k+3));
    if any(L == validLens)
        if n-k+1 >= L
            qCand = localParseQActual(rxBuffer(k:k+L-1));
            if localIsValidQ(qCand)
                idx = k; foundLen = L; return;
            end
        end
    end
end
end

function q = localParseQActual(packet)
qOffset = 253;
q = zeros(1,6);
for i = 1:6
    i0 = qOffset + (i-1)*8;
    q(i) = localBeDouble(packet(i0:(i0+7)));
end
end

function tf = localIsValidQ(q)
tf = isvector(q) && numel(q)==6 && all(isfinite(q)) && all(abs(q)<20);
end

function v = localBeInt32(b)
v = double(b(1))*16777216 + double(b(2))*65536 + double(b(3))*256 + double(b(4));
end

function d = localBeDouble(b)
d = typecast(uint8(b(end:-1:1)), 'double');
end
