function main_gui_3d
%MAIN_GUI_3D 3D GUI for viewing the scoop trajectory inside the basin.

state = build_gui_state();

state.robot = load_environment_model(state.paths.environmentUrdf);

fig = uifigure('Name', '三维铲泥轨迹规划', 'Position', [100 80 1320 760]);
mainGrid = uigridlayout(fig, [1 2], ...
    'ColumnWidth', {340, '1x'}, ...
    'Padding', [10 10 10 10], ...
    'ColumnSpacing', 10);

controlPanel = uipanel(mainGrid, 'Title', '参数与视图');
controlGrid = uigridlayout(controlPanel, [26 1], ...
    'RowHeight', {22, 30, 20, 22, 30, 20, 22, 30, 20, 22, 30, 20, ...
                  22, 30, 20, 22, 30, 20, 22, 22, 28, 28, 30, '1x', 22, 22}, ...
    'Padding', [10 10 10 10], ...
    'RowSpacing', 6);

axesPanel = uipanel(mainGrid, 'Title', '三维场景');
ax = axes(axesPanel, 'Units', 'normalized', 'Position', [0.03 0.04 0.94 0.92]);
ax.Box = 'on';
ax.FontSize = 11;

state.ui.ax = ax;
state.ui.fig = fig;

[state.ui.sWallOffset, state.ui.valWallOffset] = addSlider(controlGrid, '距左侧壁距离 (m)', ...
    [state.envGeom.basin.ySafetyMargin state.envGeom.basin.innerWidthY - state.envGeom.basin.ySafetyMargin], state.params.leftWallOffset);
[state.ui.sMudHeight, state.ui.valMudHeight] = addSlider(controlGrid, '泥面高度 hMud (mm)', [0.00 0.18], state.params.mudHeight);
[state.ui.sApproachLen, state.ui.valApproachLen] = addSlider(controlGrid, '起始运动斜线长度 L (m)', [0.00 0.30], state.params.approachLen);
[state.ui.sTheta, state.ui.valTheta] = addSlider(controlGrid, '入泥角度 theta (deg)', [-60 -5], state.params.thetaDeg);
[state.ui.sDepth, state.ui.valDepth] = addSlider(controlGrid, '入泥深度 d (垂直, m)', [0.01 0.10], state.params.depth);
[state.ui.sXPlane, state.ui.valXPlane] = addSlider(controlGrid, '轨迹平面 X 位置 xPlane (m)', ...
    [state.envGeom.basin.xPlaneMin state.envGeom.basin.xPlaneMax], state.params.xPlane);

state.ui.lblLeft = uilabel(controlGrid, 'Text', '', 'FontSize', 11);
state.ui.lblRight = uilabel(controlGrid, 'Text', '', 'FontSize', 11);

viewRow = uigridlayout(controlGrid, [1 2], 'ColumnWidth', {70, '1x'}, 'Padding', [0 0 0 0]);
uilabel(viewRow, 'Text', '视角', 'FontSize', 11);
state.ui.ddView = uidropdown(viewRow, ...
    'Items', {'Isometric', 'Top', 'Front', 'Side'}, ...
    'Value', 'Isometric');

cbRow = uigridlayout(controlGrid, [1 2], 'ColumnWidth', {'1x', '1x'}, 'Padding', [0 0 0 0]);
state.ui.cbWorld = uicheckbox(cbRow, 'Text', '显示世界坐标系', 'Value', true);
state.ui.cbArrows = uicheckbox(cbRow, 'Text', '显示姿态箭头', 'Value', true);

state.ui.btnExport = uibutton(controlGrid, 'push', ...
    'Text', '导出参数为 YAML', ...
    'ButtonPushedFcn', @(src, evt) exportYaml(fig));

state.ui.lblInfo = uilabel(controlGrid, ...
    'Text', sprintf(['轨迹说明:\n' ...
                     '轨迹位于固定 X 的 YOZ 平面内。\n' ...
                     '入泥点始终位于泥面上。\n' ...
                     '起始运动点由斜线长度和入泥角反推。\n' ...
                     '入泥深度 d 表示相对泥面的垂直下扎深度。\n' ...
                     '随后泥下圆弧转平，再沿 Z 轴竖直抬升。\n' ...
                     '淡黄色面表示泥面，黄色半透明面表示当前 YOZ 截面。']), ...
    'FontSize', 11, ...
    'HorizontalAlignment', 'left', ...
    'VerticalAlignment', 'top');
state.ui.lblInfo.Layout.Row = [24 26];

fig.UserData = state;

registerSliderCallbacks(fig, state.ui.sWallOffset, 'leftWallOffset');
registerSliderCallbacks(fig, state.ui.sMudHeight, 'mudHeight');
registerSliderCallbacks(fig, state.ui.sApproachLen, 'approachLen');
registerSliderCallbacks(fig, state.ui.sTheta, 'thetaDeg');
registerSliderCallbacks(fig, state.ui.sDepth, 'depth');
registerSliderCallbacks(fig, state.ui.sXPlane, 'xPlane');

state.ui.ddView.ValueChangedFcn = @(src, evt) onPresetViewChanged(fig);
state.ui.cbWorld.ValueChangedFcn = @(src, evt) refreshScene(fig, false);
state.ui.cbArrows.ValueChangedFcn = @(src, evt) refreshScene(fig, false);

refreshScene(fig, true);
end

function [slider, valueLabel] = addSlider(parent, labelText, limits, value)
uilabel(parent, 'Text', labelText, 'FontSize', 12);
slider = uislider(parent, ...
    'Limits', limits, ...
    'Value', value, ...
    'MajorTicks', round(linspace(limits(1), limits(2), 5), 3), ...
    'MinorTicks', []);
valueLabel = uilabel(parent, 'Text', '', 'FontSize', 11);
end

function registerSliderCallbacks(fig, slider, fieldName)
slider.ValueChangingFcn = @(src, evt) onSliderChanging(fig, fieldName, evt.Value);
slider.ValueChangedFcn = @(src, evt) onSliderChanged(fig, fieldName, src.Value);
end

function onSliderChanging(fig, fieldName, value)
state = fig.UserData;
state.params.(fieldName) = value;
fig.UserData = state;
refreshScene(fig, false);
end

function onSliderChanged(fig, fieldName, value)
state = fig.UserData;
state.params.(fieldName) = value;
fig.UserData = state;
refreshScene(fig, false);
end

function onPresetViewChanged(fig)
refreshScene(fig, true);
end

function refreshScene(fig, usePresetView)
state = fig.UserData;
state.envGeom.basin.mudSurfaceZ = state.envGeom.basin.floorZ + state.params.mudHeight;
state.traj = generate_trajectory_3d(state.params, state.envGeom);
state.ui.cameraState = resolveCameraState(state, usePresetView);
fig.UserData = state;
updateValueLabels(state);
render_scene(state, state.ui.cameraState);
fig.UserData = state;
end

function cameraState = resolveCameraState(state, usePresetView)
if nargin < 2
    usePresetView = false;
end

if usePresetView
    cameraState = struct('mode', 'preset', 'value', getPresetView(state.ui.ddView.Value));
    return;
end

if isgraphics(state.ui.ax)
    cameraState = captureCameraState(state.ui.ax);
    if ~isempty(cameraState)
        return;
    end
end

cameraState = state.ui.cameraState;
end

function azEl = getPresetView(viewName)
switch viewName
    case 'Top'
        azEl = [0, 90];
    case 'Front'
        azEl = [0, 0];
    case 'Side'
        azEl = [90, 0];
    otherwise
        azEl = [135, 20];
end
end

function cameraState = captureCameraState(ax)
try
    cameraState = struct( ...
        'mode', 'camera', ...
        'position', ax.CameraPosition, ...
        'target', ax.CameraTarget, ...
        'upVector', ax.CameraUpVector, ...
        'viewAngle', ax.CameraViewAngle);
catch
    cameraState = [];
end
end

function updateValueLabels(state)
state.ui.valWallOffset.Text = sprintf('当前值: %.3f m', state.params.leftWallOffset);
state.ui.valMudHeight.Text = sprintf('当前值: %.0f mm', 1000 * state.params.mudHeight);
state.ui.valApproachLen.Text = sprintf('当前值: %.3f m', state.params.approachLen);
state.ui.valTheta.Text = sprintf('当前值: %.1f deg', state.params.thetaDeg);
state.ui.valDepth.Text = sprintf('当前值: %.3f m', state.params.depth);
state.ui.valXPlane.Text = sprintf('当前值: %.3f m', state.params.xPlane);
state.ui.lblLeft.Text = sprintf('距 X- 侧壁距离: %.3f m', state.traj.distLeft);
state.ui.lblRight.Text = sprintf('距 X+ 侧壁距离: %.3f m', state.traj.distRight);
end

function exportYaml(fig)
state = fig.UserData;
yamlPath = fullfile(fileparts(mfilename('fullpath')), 'trajectory_params_3d.yaml');
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
    '  source_gui: "main_gui_3d"'
    'parameters:'
    sprintf('  left_wall_offset: %.6f', state.params.leftWallOffset)
    sprintf('  mud_height: %.6f', state.params.mudHeight)
    sprintf('  approach_len: %.6f', state.params.approachLen)
    sprintf('  theta_deg: %.6f', state.params.thetaDeg)
    sprintf('  depth: %.6f', state.params.depth)
    sprintf('  x_plane: %.6f', state.params.xPlane)
    'derived_values:'
    sprintf('  approach_start_point_xyz: [%.6f, %.6f, %.6f]', traj.pApproachStart(1), traj.pApproachStart(2), traj.pApproachStart(3))
    sprintf('  entry_point_xyz: [%.6f, %.6f, %.6f]', traj.pEntry(1), traj.pEntry(2), traj.pEntry(3))
    sprintf('  arc_end_point_xyz: [%.6f, %.6f, %.6f]', traj.pArcEnd(1), traj.pArcEnd(2), traj.pArcEnd(3))
    sprintf('  approach_length: %.6f', traj.approachLen)
    sprintf('  arc_radius: %.6f', traj.arcRadius)
    sprintf('  vertical_penetration: %.6f', traj.verticalPenetration)
    sprintf('  dist_x_negative_wall: %.6f', traj.distLeft)
    sprintf('  dist_x_positive_wall: %.6f', traj.distRight)
    'meanings:'
    '  left_wall_offset: "入泥点到左侧壁的水平距离，单位 m"'
    '  mud_height: "泥面距盆底的高度，单位 m"'
    '  approach_len: "起始运动点到入泥点的斜线长度，单位 m"'
    '  theta_deg: "入泥角，负值表示向下切入，单位 deg"'
    '  depth: "入泥点相对泥面的实际垂直下扎深度，单位 m"'
    '  x_plane: "当前 YOZ 轨迹所在的世界 X 截面位置，单位 m"'
    '  approach_start_point_xyz: "三维空间中的起始运动点 [X, Y, Z]"'
    '  entry_point_xyz: "三维空间中的入泥点 [X, Y, Z]"'
    '  arc_end_point_xyz: "三维空间中的圆弧转平结束点 [X, Y, Z]"'
    '  approach_length: "当前起始运动斜线长度，单位 m"'
    '  arc_radius: "根据入泥角和垂直入泥深度反算得到的圆弧半径，单位 m"'
    '  vertical_penetration: "圆弧终点相对泥面的实际垂直下扎深度，单位 m"'
    '  dist_x_negative_wall: "轨迹平面到 X- 侧壁内表面的距离，单位 m"'
    '  dist_x_positive_wall: "轨迹平面到 X+ 侧壁内表面的距离，单位 m"'
    };

yamlText = sprintf('%s\n', lines{:});
end
