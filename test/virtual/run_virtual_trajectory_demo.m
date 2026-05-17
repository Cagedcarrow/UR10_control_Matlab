function result = run_virtual_trajectory_demo(robot, mdl, demoCfg)
% 运行虚拟轨迹示例（不依赖实机）

if nargin < 3 || isempty(demoCfg)
    demoCfg = evalin('base','virtual_ur10_cfg');
end

[tVec, qCmd] = interp_joint_traj(demoCfg.demo.waypointsQ, demoCfg.demo.waypointTime, demoCfg.sampleTime, demoCfg.jointLimitsRad);

qInput = timeseries(qCmd, tVec(:));
in = Simulink.SimulationInput(mdl);
in = in.setVariable('q_cmd_ts', qInput);
in = in.setModelParameter('StopTime', num2str(tVec(end)));

if ~bdIsLoaded(mdl)
    load_system(mdl);
end
set_param([mdl '/q_cmd'],'VariableName','q_cmd_ts');

simOut = sim(in);
y = simOut.yout;
if isa(y, 'Simulink.SimulationData.Dataset')
    qActual = y.getElement(1).Values.Data;
    tActual = y.getElement(1).Values.Time;
else
    qActual = y.signals.values;
    tActual = y.time;
end

n = size(qActual,1);
tcpPos = zeros(n,3);
for k = 1:n
    [~, tcpPos(k,:), ~] = fk_tcp_pose(robot, qActual(k,:));
end

result = struct();
result.tCmd = tVec(:);
result.qCmd = qCmd;
result.tActual = tActual;
result.qActual = qActual;
result.tcpPos = tcpPos;

figure('Name','UR10 虚拟轨迹 Demo','Color','w');
tiledlayout(2,1);
nexttile;
plot(tActual, qActual, 'LineWidth', 1.2);
grid on;
xlabel('t (s)'); ylabel('q (rad)'); title('关节角响应');
legend({'J1','J2','J3','J4','J5','J6'}, 'Location','eastoutside');
nexttile;
plot3(tcpPos(:,1), tcpPos(:,2), tcpPos(:,3), 'b-', 'LineWidth', 1.5);
grid on; axis equal;
xlabel('X'); ylabel('Y'); zlabel('Z');
title('TCP 轨迹');

fprintf('[INFO] 虚拟轨迹demo运行完成。样本点: %d\n', n);
end
