function ui = ur10_gui_control_virtual(robot, cfg, mdl)
% 纯虚拟UR10控制界面（检查可达性/预执行/正式执行）

baseDirLocal = fileparts(mfilename('fullpath'));
addpath(baseDirLocal);
addpath(fullfile(baseDirLocal,'utils'));

if nargin < 1 || isempty(robot)
    cfg = init_virtual_ur10(baseDirLocal);
    [robot, ~] = build_ur10_virtual_robot(cfg);
    mdl = create_or_load_virtual_simulink_model(robot, cfg);
elseif nargin < 2 || isempty(cfg)
    cfg = init_virtual_ur10(baseDirLocal);
end

state = struct();
state.mode = '可编辑';
cfg.jointLimitsRad = getRobotJointLimits(robot, cfg.jointLimitsRad);
state.qCurrent = cfg.homeQ;
state.qTarget = cfg.homeQ;
state.robot = robot;
state.cfg = cfg;
state.model = mdl;
state.isReachable = false;
state.rrtPath = zeros(0,6);
state.previewTcpPath = zeros(0,3);
state.execTcpPath = zeros(0,3);

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
        'Value',0, 'Callback',@(src,~)onSlider(i,src));
    ui.sliderTxt(i) = uicontrol(ui.fig,'Style','text','Position',[1270 y 70 22], ...
        'String','0.00','BackgroundColor','w');
end

renderScene(true);
updateTcpText();

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
                error('RRT未找到可行路径。');
            end
            state.rrtPath = pathDense;
            state.isReachable = true;
            ui.btnPreview.Enable = 'on';
            ui.btnExec.Enable = 'on';
            setMode('可执行');
            iterTxt = localGetIterText(info);
            setMsg(sprintf('可达性检查通过（RRT路径点: %d%s）。', size(pathDense,1), iterTxt));
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
            simData.tcpPos = out.tcpPos;
            assignin('base','virtual_ur10_sim_data',simData);
            plotExecutionSignals(simData);

            setMode('停止/完成');
            setMsg('正式执行完成，已更新 virtual_ur10_sim_data（t/q/dq/tcpPos）。');
            renderScene(false);
            updateTcpText();
        catch ME
            setMode('可编辑');
            setMsg(['正式执行失败：' ME.message]);
        end
    end

    function [tVec, qCmd] = expandRRTPathToCmd(rrtPath)
        if isempty(rrtPath) || size(rrtPath,1) < 2
            error('RRT路径为空，请先检查可达性。');
        end
        segLen = vecnorm(diff(rrtPath,1,1),2,2);
        dist = [0; cumsum(segLen)];
        totalT = max(1.0, dist(end)*0.8);
        wt = (dist ./ max(dist(end), eps)) * totalT;
        [tVec, qCmd] = interp_joint_traj(rrtPath, wt', cfg.sampleTime, cfg.jointLimitsRad);
    end

    function [pathDense, info] = planRRTPath(q0, q1)
        planner = manipulatorRRT(state.robot, {});
        planner.MaxConnectionDistance = 0.25;
        planner.ValidationDistance = 0.05;
        planner.MaxIterations = 5000;
        planner.IgnoreSelfCollision = false;
        planner.SkippedSelfCollisions = 'parent';

        [pathRaw, info] = plan(planner, q0, q1);
        if isempty(pathRaw)
            pathDense = [];
            return;
        end
        pathShort = shorten(planner, pathRaw, 40);
        pathDense = interpolate(planner, pathShort, 120);
    end

    function out = runSegmentSim(tVec, qCmd)
        qInput = timeseries(qCmd, tVec(:));
        in = Simulink.SimulationInput(state.model);
        in = in.setVariable('q_cmd_ts', qInput);
        in = in.setModelParameter('StopTime', num2str(tVec(end)));
        simOut = sim(in);

        y = simOut.yout;
        if isa(y, 'Simulink.SimulationData.Dataset')
            sig = y.getElement(1).Values;
            tActual = sig.Time;
            qActual = sig.Data;
        else
            tActual = y.time;
            qActual = y.signals.values;
        end

        dq = gradient(qActual, max(cfg.sampleTime, eps));
        n = size(qActual,1);
        tcp = zeros(n,3);
        for k = 1:n
            [~, tcp(k,:), ~] = fk_tcp_pose(state.robot, qActual(k,:));
        end

        out = struct('tActual',tActual,'qActual',qActual,'dqActual',dq,'tcpPos',tcp);
    end

    function playRobot(qSeries, clearPreview)
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
        view(ui.ax,cfg.guiView(1),cfg.guiView(2));
        drawnow;
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

    function plotExecutionSignals(simData)
        figName = 'UR10 正式执行关节数据';
        oldFig = findall(groot, 'Type', 'figure', 'Name', figName);
        if ~isempty(oldFig)
            try
                delete(oldFig);
            catch
            end
        end

        fig = figure('Name', figName, 'NumberTitle', 'off', 'Color', 'w', ...
            'Position', [220 120 980 700]);
        tl = tiledlayout(fig, 2, 1, 'Padding', 'compact', 'TileSpacing', 'compact');

        nexttile(tl, 1);
        plot(simData.t, simData.q, 'LineWidth', 1.2);
        grid on;
        xlabel('t (s)');
        ylabel('q (rad)');
        title('六轴关节角度');
        legend({'J1','J2','J3','J4','J5','J6'}, 'Location', 'eastoutside');

        nexttile(tl, 2);
        plot(simData.t, simData.dq, 'LineWidth', 1.2);
        grid on;
        xlabel('t (s)');
        ylabel('dq (rad/s)');
        title('六轴关节速度');
        legend({'J1','J2','J3','J4','J5','J6'}, 'Location', 'eastoutside');
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
