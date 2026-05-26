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
controlGrid = uigridlayout(controlPanel, [28 1], ...
    'RowHeight', {22, 30, 20, 22, 30, 20, 22, 30, 20, 22, 30, 20, ...
                  22, 30, 20, 22, 30, 20, 22, 30, 20, 22, 30, 20, ...
                  22, 22, 28, 28, '1x', 22, 22}, ...
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
[state.ui.sTheta, state.ui.valTheta] = addSlider(controlGrid, '入泥角度 theta (deg)', [-60 -5], state.params.thetaDeg);
[state.ui.sDepth, state.ui.valDepth] = addSlider(controlGrid, '入泥深度 d (m)', [0.02 0.08], state.params.depth);
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

state.ui.lblInfo = uilabel(controlGrid, ...
    'Text', sprintf(['轨迹说明:\n' ...
                     '轨迹位于固定 X 的 YOZ 平面内。\n' ...
                     '入泥点始终位于泥面上。\n' ...
                     '最前面一段按固定长度沿入泥方向反向延伸。\n' ...
                     '随后泥下圆弧转平，再沿 Z 轴竖直抬升。\n' ...
                     '淡黄色面表示泥面，黄色半透明面表示当前 YOZ 截面。']), ...
    'FontSize', 11, ...
    'HorizontalAlignment', 'left', ...
    'VerticalAlignment', 'top');
state.ui.lblInfo.Layout.Row = [23 25];

fig.UserData = state;

registerSliderCallbacks(fig, state.ui.sWallOffset, 'leftWallOffset');
registerSliderCallbacks(fig, state.ui.sMudHeight, 'mudHeight');
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
state.ui.valTheta.Text = sprintf('当前值: %.1f deg', state.params.thetaDeg);
state.ui.valDepth.Text = sprintf('当前值: %.3f m', state.params.depth);
state.ui.valXPlane.Text = sprintf('当前值: %.3f m', state.params.xPlane);
state.ui.lblLeft.Text = sprintf('距 X- 侧壁距离: %.3f m', state.traj.distLeft);
state.ui.lblRight.Text = sprintf('距 X+ 侧壁距离: %.3f m', state.traj.distRight);
end
