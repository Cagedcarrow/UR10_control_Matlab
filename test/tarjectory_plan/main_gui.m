function main_gui
% 主界面：二维铲尖轨迹参数化演示（R2025a兼容）

fig = uifigure('Name','二维装载轨迹规划','Position',[100 100 980 620]);

ax = uiaxes(fig,'Position',[330 70 620 520]);
ax.Box = 'on';
ax.XGrid = 'on';
ax.YGrid = 'on';
ax.FontSize = 11;
title(ax,'铲尖二维轨迹');
xlabel(ax,'X (m)');
ylabel(ax,'Y (m)');

uilabel(fig,'Position',[25 550 260 22],'Text','入泥角度 \theta (deg)','FontSize',12);
sTheta = uislider(fig, ...
    'Position',[25 540 260 3], ...
    'Limits',[-60 -5], ...
    'Value',-30, ...
    'MajorTicks',[-60 -50 -40 -30 -20 -10 -5], ...
    'MinorTicks',[]);
valTheta = uilabel(fig,'Position',[25 515 260 22],'Text','当前值: -30.0 deg','FontSize',11);

uilabel(fig,'Position',[25 460 260 22],'Text','入泥深度 d (m)','FontSize',12);
sDepth = uislider(fig, ...
    'Position',[25 450 260 3], ...
    'Limits',[0.05 0.60], ...
    'Value',0.25, ...
    'MajorTicks',[0.05 0.15 0.25 0.35 0.45 0.55 0.60], ...
    'MinorTicks',[]);
valDepth = uilabel(fig,'Position',[25 425 260 22],'Text','当前值: 0.250 m','FontSize',11);

uilabel(fig,'Position',[25 370 260 22],'Text','底部旋转角 \phi (deg)','FontSize',12);
sPhi = uislider(fig, ...
    'Position',[25 360 260 3], ...
    'Limits',[10 140], ...
    'Value',65, ...
    'MajorTicks',[10 30 50 70 90 110 130 140], ...
    'MinorTicks',[]);
valPhi = uilabel(fig,'Position',[25 335 260 22],'Text','当前值: 65.0 deg','FontSize',11);

uilabel(fig,'Position',[25 265 280 54], ...
    'Text',sprintf(['轨迹阶段:\n' ...
                     '1) 直线切入\n' ...
                     '2) 圆弧旋转装载\n' ...
                     '3) 抬升退出']), ...
    'FontSize',11);

uilabel(fig,'Position',[25 220 280 36], ...
    'Text','可直接替换为 UR10 末端位姿控制输入。', ...
    'FontSize',11);

state.ax = ax;
state.sTheta = sTheta;
state.sDepth = sDepth;
state.sPhi = sPhi;
state.valTheta = valTheta;
state.valDepth = valDepth;
state.valPhi = valPhi;

fig.UserData = state;

sTheta.ValueChangingFcn = @(src,evt) onSliderChanging(fig, 'theta', evt.Value);
sDepth.ValueChangingFcn = @(src,evt) onSliderChanging(fig, 'depth', evt.Value);
sPhi.ValueChangingFcn = @(src,evt) onSliderChanging(fig, 'phi', evt.Value);

sTheta.ValueChangedFcn = @(src,evt) onSliderChanged(fig, 'theta', src.Value);
sDepth.ValueChangedFcn = @(src,evt) onSliderChanged(fig, 'depth', src.Value);
sPhi.ValueChangedFcn = @(src,evt) onSliderChanged(fig, 'phi', src.Value);

refreshPlot(fig);

end

function onSliderChanging(fig, name, val)
state = fig.UserData;
switch name
    case 'theta'
        state.sTheta.Value = val;
    case 'depth'
        state.sDepth.Value = val;
    case 'phi'
        state.sPhi.Value = val;
end
fig.UserData = state;
refreshPlot(fig);
end

function onSliderChanged(fig, name, val)
state = fig.UserData;
switch name
    case 'theta'
        state.sTheta.Value = val;
    case 'depth'
        state.sDepth.Value = val;
    case 'phi'
        state.sPhi.Value = val;
end
fig.UserData = state;
refreshPlot(fig);
end

function refreshPlot(fig)
state = fig.UserData;

thetaDeg = state.sTheta.Value;
d = state.sDepth.Value;
phiDeg = state.sPhi.Value;

state.valTheta.Text = sprintf('当前值: %.1f deg', thetaDeg);
state.valDepth.Text = sprintf('当前值: %.3f m', d);
state.valPhi.Text = sprintf('当前值: %.1f deg', phiDeg);

traj = generate_trajectory(thetaDeg, d, phiDeg);
plot_trajectory(state.ax, traj);

fig.UserData = state;
end
