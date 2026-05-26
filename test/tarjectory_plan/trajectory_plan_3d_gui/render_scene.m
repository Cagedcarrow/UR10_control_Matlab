function render_scene(state, cameraState)
%RENDER_SCENE Draw the environment, 3D trajectory, and overlays.

ax = state.ui.ax;
traj = state.traj;
envGeom = state.envGeom;

cla(ax);
show(state.robot, 'Parent', ax, ...
    'Visuals', 'on', ...
    'Collisions', 'off', ...
    'Frames', 'off', ...
    'PreservePlot', false, ...
    'FastUpdate', true);
hold(ax, 'on');

plot3(ax, traj.all(:, 1), traj.all(:, 2), traj.all(:, 3), '.-', ...
    'Color', [0.12 0.45 0.95], 'LineWidth', 1.6, 'MarkerSize', 10);

plot3(ax, traj.pApproachStart(1), traj.pApproachStart(2), traj.pApproachStart(3), 's', ...
    'Color', [0.25 0.25 0.25], 'MarkerFaceColor', [0.85 0.85 0.85], 'MarkerSize', 6);
plot3(ax, traj.pEntry(1), traj.pEntry(2), traj.pEntry(3), 'ko', ...
    'MarkerFaceColor', 'k', 'MarkerSize', 6);
plot3(ax, traj.pArcEnd(1), traj.pArcEnd(2), traj.pArcEnd(3), 'o', ...
    'Color', [0.85 0.25 0.25], 'MarkerFaceColor', [0.85 0.25 0.25], 'MarkerSize', 6);

drawPlanePatch(ax, envGeom.basin, traj.xPlane);
drawMudSurfacePatch(ax, envGeom.basin);
drawMudSurfaceLine(ax, envGeom.basin, traj.xPlane);

if state.ui.cbWorld.Value
    drawWorldAxes(ax);
end

if state.ui.cbArrows.Value
    drawTrajectoryArrows(ax, traj);
end

axis(ax, 'equal');
axis(ax, 'vis3d');
grid(ax, 'on');
xlabel(ax, 'X (m)');
ylabel(ax, 'Y (m)');
zlabel(ax, 'Z (m)');
xlim(ax, [-0.75, 0.75]);
ylim(ax, [-0.55, 0.55]);
zlim(ax, [-0.25, 0.75]);
applyCameraState(ax, cameraState);
title(ax, sprintf(['YOZ 平面铲泥轨迹 | x = %.3f m | 距 X- = %.3f m | 距 X+ = %.3f m\n' ...
    '距左侧壁 = %.3f m | hMud = %.0f mm | L = %.3f m | \\theta = %.1f deg | d_{vertical} = %.3f m'], ...
    traj.xPlane, traj.distLeft, traj.distRight, ...
    traj.leftWallOffset, 1000 * (traj.mudSurfaceZ - envGeom.basin.floorZ), traj.approachLen, ...
    traj.thetaDeg, traj.depth));
drawnow limitrate nocallbacks;

hold(ax, 'off');
end

function drawTrajectoryArrows(ax, traj)
idx = unique(round(linspace(1, size(traj.all, 1), 14)));
arrowLen = 0.05;
dirs = arrowLen * traj.tangent3d(idx, :);
quiver3(ax, ...
    traj.all(idx, 1), traj.all(idx, 2), traj.all(idx, 3), ...
    dirs(:, 1), dirs(:, 2), dirs(:, 3), 0, ...
    'Color', [0.18 0.18 0.18], 'LineWidth', 1.0, 'MaxHeadSize', 1.4);
end

function drawPlanePatch(ax, basin, xPlane)
xPatch = [xPlane, xPlane, xPlane, xPlane];
yPatch = [basin.innerYMin, basin.innerYMax, basin.innerYMax, basin.innerYMin];
zPatch = [basin.floorZ, basin.floorZ, basin.rimZ, basin.rimZ];
patch(ax, xPatch, yPatch, zPatch, [0.95 0.70 0.10], ...
    'FaceAlpha', 0.06, 'EdgeColor', [0.85 0.55 0.10], 'LineStyle', '--', 'LineWidth', 1.0);
end

function drawMudSurfacePatch(ax, basin)
xPatch = [basin.innerXMin, basin.innerXMax, basin.innerXMax, basin.innerXMin];
yPatch = [basin.innerYMin, basin.innerYMin, basin.innerYMax, basin.innerYMax];
zPatch = basin.mudSurfaceZ * ones(1, 4);
patch(ax, xPatch, yPatch, zPatch, [0.97 0.90 0.55], ...
    'FaceAlpha', 0.32, 'EdgeColor', [0.85 0.74 0.22], 'LineWidth', 1.0);
end

function drawMudSurfaceLine(ax, basin, xPlane)
plot3(ax, [xPlane xPlane], [basin.innerYMin basin.innerYMax], ...
    [basin.mudSurfaceZ basin.mudSurfaceZ], '--', ...
    'Color', [0.80 0.62 0.12], 'LineWidth', 1.4);
end

function drawWorldAxes(ax)
origin = [0.0, 0.0, 0.0];
len = 0.18;
quiver3(ax, origin(1), origin(2), origin(3), len, 0, 0, 0, ...
    'Color', [0.90 0.15 0.10], 'LineWidth', 1.7, 'MaxHeadSize', 0.7);
quiver3(ax, origin(1), origin(2), origin(3), 0, len, 0, 0, ...
    'Color', [0.10 0.65 0.25], 'LineWidth', 1.7, 'MaxHeadSize', 0.7);
quiver3(ax, origin(1), origin(2), origin(3), 0, 0, len, 0, ...
    'Color', [0.15 0.35 0.95], 'LineWidth', 1.7, 'MaxHeadSize', 0.7);
text(ax, len * 1.1, 0, 0, 'X', 'Color', [0.90 0.15 0.10], 'FontWeight', 'bold');
text(ax, 0, len * 1.1, 0, 'Y', 'Color', [0.10 0.65 0.25], 'FontWeight', 'bold');
text(ax, 0, 0, len * 1.1, 'Z', 'Color', [0.15 0.35 0.95], 'FontWeight', 'bold');
end

function applyCameraState(ax, cameraState)
if isempty(cameraState) || ~isstruct(cameraState) || ~isfield(cameraState, 'mode')
    view(ax, 135, 20);
    return;
end

switch cameraState.mode
    case 'preset'
        view(ax, cameraState.value(1), cameraState.value(2));
    case 'camera'
        ax.CameraPosition = cameraState.position;
        ax.CameraTarget = cameraState.target;
        ax.CameraUpVector = cameraState.upVector;
        ax.CameraViewAngle = cameraState.viewAngle;
    otherwise
        view(ax, 135, 20);
end
end
