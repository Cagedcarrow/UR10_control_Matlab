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
[state.ui.sApproachLen, state.ui.valApproachLen] = addSliderBlock(fig, 25, 460, '起始运动斜线长度 L (m)', [0.00 0.30], state.params.approachLen);
[state.ui.sTheta, state.ui.valTheta] = addSliderBlock(fig, 25, 390, '入泥角度 theta (deg)', [-60 -5], state.params.thetaDeg);
[state.ui.sDepth, state.ui.valDepth] = addSliderBlock(fig, 25, 320, '入泥深度 d (垂直, m)', [0.01 0.1], state.params.depth);

state.ui.lblLen = uilabel(fig, 'Position', [25 250 300 22], 'Text', '', 'FontSize', 11);
state.ui.lblProjX = uilabel(fig, 'Position', [25 225 300 22], 'Text', '', 'FontSize', 11);
state.ui.lblProjZ = uilabel(fig, 'Position', [25 200 300 22], 'Text', '', 'FontSize', 11);

state.ui.btnExport = uibutton(fig, 'push', ...
    'Position', [25 155 280 30], ...
    'Text', '导出参数为 YAML', ...
    'ButtonPushedFcn', @(src, evt) exportYaml(fig));

state.ui.lblInfo = uilabel(fig, ...
    'Position', [25 28 300 118], ...
    'Text', sprintf(['查看说明:\n' ...
                     '当前图只用于查看二维轨迹形状。\n' ...
                     '入泥点始终位于泥面上。\n' ...
                     '起始运动点由斜线长度和入泥角反推。\n' ...
                     '入泥深度 d 表示相对泥面的垂直下扎深度。\n' ...
                     '滑条上下限定义在本文件和 build_gui_state.m。\n' ...
                     '蓝色点线是当前原生 YOZ 轨迹。']), ...
    'FontSize', 11);

state.ui.ax = ax;
state.ui.fig = fig;
fig.UserData = state;

registerSliderCallbacks(fig, state.ui.sWallOffset, 'leftWallOffset');
registerSliderCallbacks(fig, state.ui.sMudHeight, 'mudHeight');
registerSliderCallbacks(fig, state.ui.sApproachLen, 'approachLen');
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
state.ui.valApproachLen.Text = sprintf('当前值: %.3f m', state.params.approachLen);
state.ui.valTheta.Text = sprintf('当前值: %.1f deg', state.params.thetaDeg);
state.ui.valDepth.Text = sprintf('当前值: %.3f m', state.params.depth);
state.ui.lblLen.Text = sprintf('斜线总长度 L: %.3f m', state.traj.approachLen);
state.ui.lblProjX.Text = sprintf('世界 X 投影: %.3f m', state.traj.approachProjX);
state.ui.lblProjZ.Text = sprintf('世界 Z 投影: %.3f m', state.traj.approachProjZ);
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

function exportYaml(fig)
state = fig.UserData;
yamlPath = fullfile(fileparts(mfilename('fullpath')), 'trajectory_params_2d.yaml');
yamlText = buildYamlText(state);

fid = fopen(yamlPath, 'w', 'n', 'UTF-8');
if fid < 0
    uialert(fig, sprintf('无法写入 YAML 文件:\n%s', yamlPath), '导出失败');
    return;
end
cleanupObj = onCleanup(@() fclose(fid));
fwrite(fid, yamlText, 'char');
clear cleanupObj;

uialert(fig, sprintf('参数已导出到:\n%s', yamlPath), '导出完成', 'Icon', 'success');
end

function yamlText = buildYamlText(state)
traj = state.traj;
timestamp = char(datetime('now', 'Format', 'yyyy-MM-dd HH:mm:ss'));

lines = {
    'metadata:'
    sprintf('  exported_at: "%s"', timestamp)
    '  source_gui: "main_gui_2d"'
    'parameters:'
    sprintf('  left_wall_offset: %.6f', state.params.leftWallOffset)
    sprintf('  mud_height: %.6f', state.params.mudHeight)
    sprintf('  approach_len: %.6f', state.params.approachLen)
    sprintf('  theta_deg: %.6f', state.params.thetaDeg)
    sprintf('  depth: %.6f', state.params.depth)
    'derived_values:'
    sprintf('  entry_point: [%.6f, %.6f]', traj.pEntry(1), traj.pEntry(2))
    sprintf('  approach_start_point: [%.6f, %.6f]', traj.pApproachStart(1), traj.pApproachStart(2))
    sprintf('  arc_end_point: [%.6f, %.6f]', traj.pArcEnd(1), traj.pArcEnd(2))
    sprintf('  approach_length: %.6f', traj.approachLen)
    sprintf('  arc_radius: %.6f', traj.arcRadius)
    sprintf('  vertical_penetration: %.6f', traj.verticalPenetration)
    sprintf('  approach_proj_x: %.6f', traj.approachProjX)
    sprintf('  approach_proj_z: %.6f', traj.approachProjZ)
    'meanings:'
    '  left_wall_offset: "入泥点到左侧壁的水平距离，单位 m"'
    '  mud_height: "泥面距盆底的高度，单位 m"'
    '  approach_len: "起始运动点到入泥点的斜线长度，单位 m"'
    '  theta_deg: "入泥角，负值表示向下切入，单位 deg"'
    '  depth: "入泥点相对泥面的实际垂直下扎深度，单位 m"'
    '  entry_point: "YOZ 截面内的入泥点 [Y, Z]"'
    '  approach_start_point: "YOZ 截面内的起始运动点 [Y, Z]"'
    '  arc_end_point: "圆弧转平结束点 [Y, Z]"'
    '  arc_radius: "根据入泥角和垂直入泥深度反算得到的圆弧半径，单位 m"'
    '  vertical_penetration: "圆弧终点相对泥面的实际垂直下扎深度，单位 m"'
    '  approach_proj_x: "当前二维/三维约定下世界 X 方向投影；二维逻辑未引入 X 位移，因此为 0"'
    '  approach_proj_z: "起始运动斜线在世界 Z 方向的投影，单位 m"'
    };

yamlText = sprintf('%s\n', lines{:});
end
