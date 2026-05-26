function main_gui_2d
%MAIN_GUI_2D 2D viewer for the current YOZ-plane trajectory logic.

state = build_gui_state();

fig = uifigure('Name', '二维 YOZ 轨迹查看', 'Position', [100 90 1040 680]);

ax = uiaxes(fig, 'Position', [360 70 640 560]);
ax.Box = 'on';
ax.XGrid = 'on';
ax.YGrid = 'on';
ax.FontSize = 11;
title(ax, 'YOZ 截面二维轨迹');
xlabel(ax, 'Y (m)');
ylabel(ax, 'Z (m)');

[state.ui.sWallOffset, state.ui.valWallOffset] = addSliderBlock(fig, 25, 600, '距左侧壁距离 (m)', ...
    [state.envGeom.basin.ySafetyMargin, state.envGeom.basin.innerWidthY - state.envGeom.basin.ySafetyMargin], ...
    state.params.leftWallOffset);
[state.ui.sMudHeight, state.ui.valMudHeight] = addSliderBlock(fig, 25, 530, '泥面高度 hMud (mm)', [0.00 0.18], state.params.mudHeight);
[state.ui.sTheta, state.ui.valTheta] = addSliderBlock(fig, 25, 460, '入泥角度 theta (deg)', [-60 -5], state.params.thetaDeg);
[state.ui.sDepth, state.ui.valDepth] = addSliderBlock(fig, 25, 390, '入泥深度 d (m)', [0.02 0.08], state.params.depth);

state.ui.lblInfo = uilabel(fig, ...
    'Position', [25 180 300 92], ...
    'Text', sprintf(['查看说明:\n' ...
                     '当前图只用于查看二维轨迹形状。\n' ...
                     '入泥点始终位于泥面上。\n' ...
                     '蓝色点线是当前原生 YOZ 轨迹。']), ...
    'FontSize', 11);

state.ui.ax = ax;
state.ui.fig = fig;
fig.UserData = state;

registerSliderCallbacks(fig, state.ui.sWallOffset, 'leftWallOffset');
registerSliderCallbacks(fig, state.ui.sMudHeight, 'mudHeight');
registerSliderCallbacks(fig, state.ui.sTheta, 'thetaDeg');
registerSliderCallbacks(fig, state.ui.sDepth, 'depth');

refreshPlot(fig);
end

function [slider, valueLabel] = addSliderBlock(fig, left, top, labelText, limits, value)
uilabel(fig, 'Position', [left top 300 22], 'Text', labelText, 'FontSize', 12);
slider = uislider(fig, ...
    'Position', [left top-10 280 3], ...
    'Limits', limits, ...
    'Value', value, ...
    'MajorTicks', round(linspace(limits(1), limits(2), 5), 3), ...
    'MinorTicks', []);
valueLabel = uilabel(fig, 'Position', [left top-35 300 22], 'Text', '', 'FontSize', 11);
end

function registerSliderCallbacks(fig, slider, fieldName)
slider.ValueChangingFcn = @(src, evt) onSliderChanging(fig, fieldName, evt.Value);
slider.ValueChangedFcn = @(src, evt) onSliderChanged(fig, fieldName, src.Value);
end

function onSliderChanging(fig, fieldName, value)
state = fig.UserData;
state.params.(fieldName) = value;
fig.UserData = state;
refreshPlot(fig);
end

function onSliderChanged(fig, fieldName, value)
state = fig.UserData;
state.params.(fieldName) = value;
fig.UserData = state;
refreshPlot(fig);
end

function refreshPlot(fig)
state = fig.UserData;
state.envGeom.basin.mudSurfaceZ = state.envGeom.basin.floorZ + state.params.mudHeight;
state.traj = generate_trajectory_yoz(state.params, state.envGeom);
updateValueLabels(state);
plotTrajectory2D(state.ui.ax, state.traj, state.envGeom.basin);
fig.UserData = state;
end

function updateValueLabels(state)
state.ui.valWallOffset.Text = sprintf('当前值: %.3f m', state.params.leftWallOffset);
state.ui.valMudHeight.Text = sprintf('当前值: %.0f mm', 1000 * state.params.mudHeight);
state.ui.valTheta.Text = sprintf('当前值: %.1f deg', state.params.thetaDeg);
state.ui.valDepth.Text = sprintf('当前值: %.3f m', state.params.depth);
end

function plotTrajectory2D(ax, traj, basin)
cla(ax);
hold(ax, 'on');

yMin = basin.innerYMin;
yMax = basin.innerYMax;
zMin = basin.floorZ;
zMax = basin.rimZ;

patch(ax, [yMin yMax yMax yMin], ...
    [zMin zMin basin.mudSurfaceZ basin.mudSurfaceZ], ...
    [0.97 0.90 0.55], ...
    'FaceAlpha', 0.32, 'EdgeColor', 'none');

plot(ax, [yMin yMin], [zMin zMax], '-', 'Color', [0.20 0.35 0.70], 'LineWidth', 1.4);
plot(ax, [yMin yMax], [zMin zMin], '-', 'Color', [0.20 0.35 0.70], 'LineWidth', 1.4);
plot(ax, [yMax yMax], [zMin zMax], '-', 'Color', [0.20 0.35 0.70], 'LineWidth', 1.4);
plot(ax, [yMin yMax], [basin.mudSurfaceZ basin.mudSurfaceZ], '--', ...
    'Color', [0.80 0.62 0.12], 'LineWidth', 1.4);

plot(ax, traj.all(:, 1), traj.all(:, 2), '.-', ...
    'Color', [0.12 0.45 0.95], 'LineWidth', 1.6, 'MarkerSize', 10);

plot(ax, traj.pApproachStart(1), traj.pApproachStart(2), 's', ...
    'Color', [0.25 0.25 0.25], 'MarkerFaceColor', [0.85 0.85 0.85], 'MarkerSize', 6);
plot(ax, traj.pEntry(1), traj.pEntry(2), 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 6);
plot(ax, traj.pArcEnd(1), traj.pArcEnd(2), 'o', 'Color', [0.85 0.25 0.25], ...
    'MarkerFaceColor', [0.85 0.25 0.25], 'MarkerSize', 6);

text(ax, yMin + 0.01, basin.mudSurfaceZ + 0.004, ...
    sprintf('泥面 hMud = %.0f mm', 1000 * (basin.mudSurfaceZ - basin.floorZ)), ...
    'Color', [0.55 0.42 0.06], 'FontSize', 10, 'FontWeight', 'bold');
text(ax, traj.pApproachStart(1), traj.pApproachStart(2) + 0.006, '起始运动点', ...
    'Color', [0.25 0.25 0.25], 'FontSize', 9, 'HorizontalAlignment', 'center');
text(ax, traj.pEntry(1), traj.pEntry(2) + 0.006, '入泥点', ...
    'Color', [0.10 0.10 0.10], 'FontSize', 9, 'HorizontalAlignment', 'center');
text(ax, traj.pArcEnd(1), traj.pArcEnd(2) + 0.006, '转平结束点', ...
    'Color', [0.85 0.25 0.25], 'FontSize', 9, 'HorizontalAlignment', 'center');

axis(ax, 'equal');
xlim(ax, [yMin - 0.03, yMax + 0.03]);
ylim(ax, [zMin - 0.01, zMax + 0.02]);
title(ax, sprintf('二维轨迹查看 | 左侧距 = %.3f m | hMud = %.0f mm | theta = %.1f deg | d = %.3f m', ...
    traj.leftWallOffset, 1000 * (basin.mudSurfaceZ - basin.floorZ), traj.thetaDeg, traj.depth));

hold(ax, 'off');
end
