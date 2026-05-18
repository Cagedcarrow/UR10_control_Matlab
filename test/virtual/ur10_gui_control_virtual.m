function ui = ur10_gui_control_virtual(robot, cfg, mdl)
% 纯虚拟UR10控制界面（检查可达性/预执行/正式执行）

baseDirLocal = fileparts(mfilename('fullpath'));
addpath(baseDirLocal);
addpath(fullfile(baseDirLocal,'utils'));

if nargin < 1 || isempty(robot)
    cfg = init_virtual_ur10(baseDirLocal);
    ensure_virtual_from_workspace_signals(cfg.sampleTime, 2);
    [robot, ~] = build_ur10_virtual_robot(cfg);
    mdl = create_or_load_virtual_simulink_model(robot, cfg);
elseif nargin < 2 || isempty(cfg)
    cfg = init_virtual_ur10(baseDirLocal);
end
ensure_virtual_from_workspace_signals(cfg.sampleTime, 2);
cfg.homeQ = loadHomeQFromCsv(cfg);

state = struct();
state.mode = '可编辑';
cfg.jointLimitsRad = getRobotJointLimits(robot, cfg.jointLimitsRad);
state.qCurrent = validate_joint_vector(cfg.homeQ, cfg.jointLimitsRad);
state.qTarget = state.qCurrent;
state.robot = robot;
state.cfg = cfg;
state.model = mdl;
state.isReachable = false;
state.rrtPath = zeros(0,6);
state.previewTcpPath = zeros(0,3);
state.execTcpPath = zeros(0,3);
state.camFixed = [];
state.isAnimating = false;

ui.fig = figure('Name','UR10 纯虚拟控制界面','NumberTitle','off','Color','w', ...
    'Position',[100 80 1380 820], 'CloseRequestFcn',@onClose);
ui.ax = axes('Parent',ui.fig,'Units','pixels','Position',[20 170 950 620]);
axis(ui.ax,'equal'); grid(ui.ax,'on'); view(ui.ax,cfg.guiView(1),cfg.guiView(2));

ui.txtMode = uicontrol(ui.fig,'Style','text','Position',[1000 770 340 24], ...
    'HorizontalAlignment','left','BackgroundColor','w','String','状态：可编辑');
ui.txtTcp = uicontrol(ui.fig,'Style','text','Position',[1000 740 340 24], ...
    'HorizontalAlignment','left','BackgroundColor','w','String','TCP：');
ui.txtMsg = uicontrol(ui.fig,'Style','text','Position',[20 130 1320 24], ...
    'HorizontalAlignment','left','BackgroundColor','w','String','消息：虚拟系统已就绪');

ui.btnReach = uicontrol(ui.fig,'Style','pushbutton','Position',[1000 690 320 34], ...
    'String','1: 检查可达性','Callback',@onReachability);
ui.btnPreview = uicontrol(ui.fig,'Style','pushbutton','Position',[1000 648 320 34], ...
    'String','2: 点击预执行','Enable','off','Callback',@onPreview);
ui.btnExec = uicontrol(ui.fig,'Style','pushbutton','Position',[1000 606 320 34], ...
    'String','3: 点击正式执行','Enable','off','Callback',@onExecute);

ui.sliders = gobjects(1,6);
ui.sliderTxt = gobjects(1,6);
for i = 1:6
    y = 520 - (i-1)*66;
    uicontrol(ui.fig,'Style','text','Position',[1000 y+26 190 22], ...
        'String',sprintf('第%d轴目标角(度)', i), 'BackgroundColor','w', 'HorizontalAlignment','left');
    ui.sliders(i) = uicontrol(ui.fig,'Style','slider','Position',[1000 y 260 22], ...
        'Min',rad2deg(cfg.jointLimitsRad(i,1)), 'Max',rad2deg(cfg.jointLimitsRad(i,2)), ...
        'Value',rad2deg(state.qTarget(i)), 'Callback',@(src,~)onSlider(i,src));
    ui.sliderTxt(i) = uicontrol(ui.fig,'Style','text','Position',[1270 y 70 22], ...
        'String','0.00','BackgroundColor','w');
end

renderScene(true);
for i = 1:6
    ui.sliderTxt(i).String = sprintf('%.2f', rad2deg(state.qTarget(i)));
end
updateTcpText();
setMsg(sprintf('初始关节角来自CSV/配置: [%.3f %.3f %.3f %.3f %.3f %.3f] rad', state.qCurrent));

    function onSlider(i, src)
        state.qTarget(i) = deg2rad(src.Value);
        ui.sliderTxt(i).String = sprintf('%.2f', src.Value);
        state.isReachable = false;
        ui.btnPreview.Enable = 'off';
        ui.btnExec.Enable = 'off';
        setMode('可编辑');
        setMsg('目标已修改，请先检查可达性。');
        renderScene(true);
    end

    function onReachability(~,~)
        try
            q0 = validate_joint_vector(state.qCurrent, cfg.jointLimitsRad);
            q1 = validate_joint_vector(state.qTarget, cfg.jointLimitsRad);
            [pathDense, info] = planRRTPath(q0, q1);
            if isempty(pathDense)
                error('未找到可行路径。');
            end
            state.rrtPath = pathDense;
            state.isReachable = true;
            ui.btnPreview.Enable = 'on';
            ui.btnExec.Enable = 'on';
            setMode('可执行');
            iterTxt = localGetIterText(info);
            setMsg(sprintf('可达性检查通过（路径点: %d%s）。', size(pathDense,1), iterTxt));
            renderScene(true);
            updateTcpText();
        catch ME
            state.isReachable = false;
            state.rrtPath = zeros(0,6);
            ui.btnPreview.Enable = 'off';
            ui.btnExec.Enable = 'off';
            setMode('可编辑');
            setMsg(['可达性检查失败：' ME.message]);
        end
    end

    function onPreview(~,~)
        if ~state.isReachable
            setMsg('请先通过可达性检查。');
            return;
        end
        try
            setMode('预执行中');
            setMsg('预执行动画中...');
            [tVec, qCmd] = expandRRTPathToCmd(state.rrtPath);
            out = runSegmentSim(tVec, qCmd);
            playRobot(out.qActual, true);

            state.previewTcpPath = out.tcpPos;
            state.qCurrent = qCmd(1,:);
            setMode('预执行完成');
            setMsg('预执行完成，已回到起点。半透明轨迹已清除，末端轨迹已保留。');
            renderScene(false);
            updateTcpText();
        catch ME
            setMode('可编辑');
            setMsg(['预执行失败：' ME.message]);
        end
    end

    function onExecute(~,~)
        if ~state.isReachable
            setMsg('请先通过可达性检查。');
            return;
        end
        try
            setMode('正式执行中');
            setMsg('正式执行中，Simulink数据更新...');
            [tVec, qCmd] = expandRRTPathToCmd(state.rrtPath);
            out = runSegmentSim(tVec, qCmd);
            playRobot(out.qActual, false);

            state.qCurrent = out.qActual(end,:);
            state.execTcpPath = out.tcpPos;
            state.previewTcpPath = zeros(0,3);
            state.isReachable = false;
            state.rrtPath = zeros(0,6);
            ui.btnPreview.Enable = 'off';
            ui.btnExec.Enable = 'off';

            simData = struct();
            simData.t = out.tActual;
            simData.q = out.qActual;
            simData.dq = out.dqActual;
            simData.tau = out.tauEst;
            simData.iEst = out.iEst;
            simData.tcpPos = out.tcpPos;
            simData.ref = out.expRef;
            assignin('base','virtual_ur10_sim_data',simData);
            openScopeWindows();

            setMode('停止/完成');
            setMsg('正式执行完成，已更新 virtual_ur10_sim_data（t/q/dq/tau/iEst/tcpPos），并弹出Simulink Scope。');
            renderScene(false);
            updateTcpText();
        catch ME
            setMode('可编辑');
            setMsg(['正式执行失败：' ME.message]);
        end
    end

    function [tVec, qCmd] = expandRRTPathToCmd(rrtPath)
        if isempty(rrtPath) || size(rrtPath,1) < 2
            error('路径为空，请先检查可达性。');
        end
        segLen = vecnorm(diff(rrtPath,1,1),2,2);
        dist = [0; cumsum(segLen)];
        totalT = max(1.0, dist(end)*0.8);
        wt = (dist ./ max(dist(end), eps)) * totalT;
        [tVec, qCmd] = interp_joint_traj(rrtPath, wt', cfg.sampleTime, cfg.jointLimitsRad);
    end

    function [pathDense, info] = planRRTPath(q0, q1)
        q0 = q0(:)';
        q1 = q1(:)';
        d = norm(q1 - q0, 2);
        n = max(2, ceil(d / 0.03));
        alpha = linspace(0, 1, n)';
        pathDense = (1 - alpha) .* q0 + alpha .* q1;
        info = struct('Iterations', 0, 'CollisionChecked', false);

        if isfield(cfg,'collision') && isfield(cfg.collision,'enable') && cfg.collision.enable
            [ok, collisionMsg] = checkPathCollision(pathDense);
            info.CollisionChecked = true;
            if ~ok
                fprintf(2, '[COLLISION] %s\n', collisionMsg);
                error(collisionMsg);
            end
        end
    end

    function [ok, msg] = checkPathCollision(pathDense)
        ok = true;
        msg = '';
        world = {};
        exemptions = normalizeExemptions();
        for k = 1:size(pathDense,1)
            qk = pathDense(k,:);
            [isColliding, pairs] = localCheckCollisionPairs(qk, world);
            if ~isColliding
                continue;
            end
            pairs = filterExemptPairs(pairs, exemptions);
            if ~isempty(pairs)
                ok = false;
                msg = buildCollisionMessage(k, pairs);
                return;
            end
        end
    end

    function msg = buildCollisionMessage(idx, pairs)
        if isempty(pairs)
            msg = sprintf('可达性碰撞：第%d个路径点发生碰撞（未返回碰撞对）。', idx);
            return;
        end
        lines = strings(1, size(pairs,1));
        for p = 1:size(pairs,1)
            a = string(pairs{p,1});
            b = string(pairs{p,2});
            lines(p) = a + " <-> " + b;
        end
        msg = sprintf('可达性碰撞：第%d个路径点发生碰撞，碰撞对: %s', idx, strjoin(lines, '; '));
    end

    function pairs = sepMatrixToPairs(sep, robotObj)
        pairs = {};
        if isempty(sep) || ~ismatrix(sep)
            return;
        end
        names = ['base', string(robotObj.BodyNames(:)')];
        hasCollision = false(1, numel(names));
        for b = 2:numel(names)
            bodyObj = robotObj.getBody(char(names(b)));
            hasCollision(b) = ~isempty(bodyObj.Collisions);
        end
        [r, c] = find(triu(isnan(sep), 1));
        out = cell(0,2);
        for t = 1:numel(r)
            % 没有碰撞几何的body不参与真实碰撞判断，NaN属于无效分离距离
            if ~hasCollision(r(t)) || ~hasCollision(c(t))
                continue;
            end
            out(end+1,1) = {char(names(r(t)))}; %#ok<AGROW>
            out(end,2) = {char(names(c(t)))}; %#ok<AGROW>
        end
        pairs = out;
    end

    function [isColliding, pairs] = localCheckCollisionPairs(qk, world)
        try
            [isColliding, sep] = checkCollision(state.robot, qk, world, ...
                'IgnoreSelfCollision', 'off', ...
                'SkippedSelfCollisions', 'parent');
            pairs = sepMatrixToPairs(sep, state.robot);
        catch
            [isColliding, sep] = checkCollision(state.robot, qk, world, ...
                'IgnoreSelfCollision', false, ...
                'SkippedSelfCollisions', 'parent');
            pairs = sepMatrixToPairs(sep, state.robot);
        end
    end

    function ex = normalizeExemptions()
        ex = strings(0,2);
        if ~isfield(cfg,'collision') || ~isfield(cfg.collision,'exemptions') || isempty(cfg.collision.exemptions)
            return;
        end
        raw = cfg.collision.exemptions;
        if iscell(raw) && numel(raw) == 2 && ischar(raw{1}) && ischar(raw{2})
            ex = [string(raw{1}) string(raw{2})];
            return;
        end
        if iscell(raw) && size(raw,2) == 2
            ex = strings(size(raw,1),2);
            for i = 1:size(raw,1)
                ex(i,1) = string(raw{i,1});
                ex(i,2) = string(raw{i,2});
            end
        end
    end

    function kept = filterExemptPairs(pairs, ex)
        if isempty(pairs) || isempty(ex)
            kept = pairs;
            return;
        end
        keepMask = true(size(pairs,1),1);
        for i = 1:size(pairs,1)
            a = string(pairs{i,1});
            b = string(pairs{i,2});
            for j = 1:size(ex,1)
                x = ex(j,1); y = ex(j,2);
                if (a == x && b == y) || (a == y && b == x)
                    keepMask(i) = false;
                    break;
                end
            end
        end
        kept = pairs(keepMask,:);
    end

    function out = runSegmentSim(tVec, qCmd)
        qInput = timeseries(qCmd, tVec(:));
        expRef = loadExperimentalReference(tVec(:), cfg);
        in = Simulink.SimulationInput(state.model);
        in = in.setVariable('q_cmd_ts', qInput);
        in = in.setVariable('q_ref_ts', timeseries(expRef.q, tVec(:)));
        in = in.setVariable('dq_ref_ts', timeseries(expRef.dq, tVec(:)));
        in = in.setVariable('tau_csv_ref_ts', timeseries(expRef.tauCsv, tVec(:)));
        in = in.setVariable('tau_from_i_ref_ts', timeseries(expRef.tauFromCurrent, tVec(:)));
        in = in.setBlockParameter([state.model '/JointSpaceMotionModel'], 'X0', mat2str(qCmd(1,:).'));
        in = in.setModelParameter('StopTime', num2str(tVec(end)));
        simOut = sim(in);

        y = simOut.yout;
        if isa(y, 'Simulink.SimulationData.Dataset')
            qSig = y.getElement(1).Values;
            dqSig = y.getElement(2).Values;
            tauSig = y.getElement(3).Values;
            iSig = y.getElement(4).Values;
            tActual = qSig.Time;
            qActual = qSig.Data;
            dqActual = dqSig.Data;
            tauEst = tauSig.Data;
            iEst = iSig.Data;
        else
            tActual = y.time;
            qActual = y.signals(1).values;
            dqActual = y.signals(2).values;
            tauEst = y.signals(3).values;
            iEst = y.signals(4).values;
        end

        n = size(qActual,1);
        tcp = zeros(n,3);
        for k = 1:n
            [~, tcp(k,:), ~] = fk_tcp_pose(state.robot, qActual(k,:));
        end

        out = struct('tActual',tActual,'qActual',qActual,'dqActual',dqActual,'tauEst',tauEst,'iEst',iEst,'tcpPos',tcp,'expRef',expRef);
    end

    function playRobot(qSeries, clearPreview)
        state.isAnimating = true;
        cAnim = onCleanup(@() setAnimating(false)); %#ok<NASGU>
        for k = 1:size(qSeries,1)
            state.qCurrent = qSeries(k,:);
            renderScene(~clearPreview);
            updateTcpText();
            pause(cfg.sampleTime*0.5);
        end
        if clearPreview
            state.qCurrent = qSeries(1,:);
            renderScene(false);
            updateTcpText();
        end
    end

    function renderScene(showGhost)
        cam = captureCamera();
        cla(ui.ax);
        hold(ui.ax,'on');
        drawGround(ui.ax, cfg.groundSize);

        show(state.robot, state.qCurrent, 'Parent', ui.ax, 'Visuals','on', 'Frames','off', ...
            'Collisions','off', 'PreservePlot', false, 'FastUpdate', true);
        p1 = findobj(ui.ax,'Type','Patch');
        set(p1, 'FaceAlpha', 1.0, 'EdgeAlpha', 0.08);

        if showGhost
            show(state.robot, state.qTarget, 'Parent', ui.ax, 'Visuals','on', 'Frames','off', ...
                'Collisions','off', 'PreservePlot', true);
            p2 = setdiff(findobj(ui.ax,'Type','Patch'), p1);
            if ~isempty(p2)
                set(p2, 'FaceAlpha', 0.20, 'EdgeAlpha', 0.03, 'FaceColor', [0.2 0.7 1.0]);
            end
        end

        if ~isempty(state.previewTcpPath)
            plot3(ui.ax, state.previewTcpPath(:,1), state.previewTcpPath(:,2), state.previewTcpPath(:,3), ...
                'm-', 'LineWidth', 1.5);
        end
        if ~isempty(state.execTcpPath)
            plot3(ui.ax, state.execTcpPath(:,1), state.execTcpPath(:,2), state.execTcpPath(:,3), ...
                'b-', 'LineWidth', 1.8);
        end

        tcp = tform2trvec(getTransform(state.robot, state.qCurrent, 'sensor_shovel_tcp'));
        plot3(ui.ax, tcp(1), tcp(2), tcp(3), 'ro', 'MarkerFaceColor','r', 'MarkerSize',7);

        hold(ui.ax,'off');
        axis(ui.ax,'equal'); grid(ui.ax,'on');
        if ~state.isAnimating
            restoreCamera(cam);
        end
        drawnow;
    end

    function setAnimating(tf)
        state.isAnimating = logical(tf);
    end

    function cam = captureCamera()
        if isempty(state.camFixed)
            cam = struct( ...
                'CameraPosition', ui.ax.CameraPosition, ...
                'CameraTarget', ui.ax.CameraTarget, ...
                'CameraUpVector', ui.ax.CameraUpVector, ...
                'CameraViewAngle', ui.ax.CameraViewAngle);
            state.camFixed = cam;
        else
            cam = struct( ...
                'CameraPosition', ui.ax.CameraPosition, ...
                'CameraTarget', ui.ax.CameraTarget, ...
                'CameraUpVector', ui.ax.CameraUpVector, ...
                'CameraViewAngle', ui.ax.CameraViewAngle);
            state.camFixed = cam;
        end
    end

    function restoreCamera(cam)
        if isempty(cam)
            return;
        end
        ui.ax.CameraPosition = cam.CameraPosition;
        ui.ax.CameraTarget = cam.CameraTarget;
        ui.ax.CameraUpVector = cam.CameraUpVector;
        ui.ax.CameraViewAngle = cam.CameraViewAngle;
    end

    function updateTcpText()
        [~, pos, rpy] = fk_tcp_pose(state.robot, state.qCurrent);
        ui.txtTcp.String = sprintf('TCP: [%.3f %.3f %.3f] m, RPY=[%.2f %.2f %.2f] deg', ...
            pos(1),pos(2),pos(3),rad2deg(rpy(1)),rad2deg(rpy(2)),rad2deg(rpy(3)));
    end

    function setMode(m)
        state.mode = m;
        ui.txtMode.String = ['状态：' m];
    end

    function setMsg(s)
        ui.txtMsg.String = ['消息：' s];
        fprintf('[INFO] %s\n', s);
    end

    function txt = localGetIterText(info)
        txt = '';
        try
            if isstruct(info) && isfield(info,'Iterations')
                txt = sprintf(', 迭代: %d', info.Iterations);
            end
        catch
        end
    end

    function openScopeWindows()
        if ~isfield(cfg,'scopeNames') || isempty(cfg.scopeNames)
            return;
        end
        for k = 1:numel(cfg.scopeNames)
            blk = [state.model '/' cfg.scopeNames{k}];
            if isempty(find_system(state.model,'SearchDepth',1,'Name',cfg.scopeNames{k}))
                continue;
            end
            try
                open_system(blk);
            catch
            end
        end
    end

    function expRef = loadExperimentalReference(tVec, cfgLocal)
        n = numel(tVec);
        expRef = struct('q',zeros(n,6),'dq',zeros(n,6),'tauCsv',zeros(n,6),'tauFromCurrent',zeros(n,6));
        csvPath = cfgLocal.experimentalCsvPath;
        if ~isfile(csvPath)
            setMsg(sprintf('实验CSV不存在，使用零参考：%s', csvPath));
            return;
        end
        try
            tbl = readtable(csvPath);
            qExp = tableToMatrixCols(tbl,'Act_q');
            dqExp = tableToMatrixCols(tbl,'Act_qd');
            iExp = tableToMatrixCols(tbl,'Act_I');
            tauCsv = tableToMatrixCols(tbl,'tau_estimated_');
            tauFromI = iExp .* reshape(cfgLocal.motorGains,1,6);

            m = min([size(qExp,1), size(dqExp,1), size(tauCsv,1), size(tauFromI,1)]);
            if m < 2
                error('实验数据长度不足。');
            end
            idx = round(linspace(1,m,n));
            expRef.q = qExp(idx,:);
            expRef.dq = dqExp(idx,:);
            expRef.tauCsv = tauCsv(idx,:);
            expRef.tauFromCurrent = tauFromI(idx,:);
        catch ME
            setMsg(['加载实验CSV失败，已回退零参考：' ME.message]);
        end
    end

    function mat = tableToMatrixCols(tbl, prefix)
        mat = zeros(height(tbl),6);
        for j = 1:6
            col = [prefix num2str(j-1)];
            if ~ismember(col, tbl.Properties.VariableNames)
                error('缺少列: %s', col);
            end
            mat(:,j) = tbl.(col);
        end
    end

    function onClose(~,~)
        if isgraphics(ui.fig)
            delete(ui.fig);
        end
    end
end

function drawGround(ax, sizeM)
[X,Y] = meshgrid(linspace(-sizeM,sizeM,2), linspace(-sizeM,sizeM,2));
Z = zeros(size(X));
surf(ax, X, Y, Z, 'FaceColor',[0.95 0.95 0.95], 'EdgeColor',[0.85 0.85 0.85], 'FaceAlpha',0.5);
end

function jointLimits = getRobotJointLimits(robot, fallbackLimits)
jointLimits = fallbackLimits;
j = 0;
for i = 1:numel(robot.Bodies)
    jt = robot.Bodies{i}.Joint;
    if strcmp(jt.Type, 'revolute')
        j = j + 1;
        lim = jt.PositionLimits;
        if numel(lim) == 2 && all(isfinite(lim)) && lim(2) > lim(1)
            jointLimits(j,:) = lim(:)';
        end
    end
end
end

function qHome = loadHomeQFromCsv(cfg)
qHome = cfg.homeQ;
if ~isfield(cfg, 'experimentalCsvPath') || ~isfile(cfg.experimentalCsvPath)
    return;
end
try
    tbl = readtable(cfg.experimentalCsvPath);
    if height(tbl) < 1
        return;
    end
    row = tbl(1, :);
    cols = {'Act_q0','Act_q1','Act_q2','Act_q3','Act_q4','Act_q5'};
    if ~all(ismember(cols, row.Properties.VariableNames))
        return;
    end
    q = [row.Act_q0, row.Act_q1, row.Act_q2, row.Act_q3, row.Act_q4, row.Act_q5];
    if numel(q) == 6 && all(isfinite(q))
        qHome = double(q);
    end
catch
end
end
