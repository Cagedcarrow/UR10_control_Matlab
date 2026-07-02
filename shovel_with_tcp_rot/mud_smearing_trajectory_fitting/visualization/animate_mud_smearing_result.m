function animate_mud_smearing_result(robot, cfg, motionConfigs, trajectory, ikResult, cameraState)
%ANIMATE_MUD_SMEARING_RESULT Display the scene, shovel surface, and target path.

if nargin < 6
    cameraState = [];
end

if cfg.visual.showAnimation
    fig = figure("Name", "mud smearing trajectory fitting", "Color", "w");
else
    fig = figure("Name", "mud smearing trajectory fitting", "Color", "w", "Visible", "off");
end
ax = axes("Parent", fig);

for frameIndex = 1:size(motionConfigs, 1)
    show(robot, motionConfigs(frameIndex, :), ...
        "Parent", ax, ...
        "Visuals", "on", ...
        "Collisions", "off", ...
        "Frames", "on", ...
        "PreservePlot", false);
    axis(ax, "equal");
    grid(ax, "on");
    if isempty(cameraState)
        if frameIndex == 1
            view(ax, 135, 20);
            cameraState = captureCameraState(ax);
        else
            restoreCameraState(ax, cameraState);
        end
    else
        restoreCameraState(ax, cameraState);
    end
    title(ax, sprintf("mud smearing trajectory (%d/%d)", frameIndex, size(motionConfigs, 1)));

    drawWallPlane(ax, trajectory.wall, cfg);
    drawTargetPath(ax, trajectory);
    drawShovelSurface(ax, robot, motionConfigs(frameIndex, :), cfg);
    drawFrameAxes(ax, robot, motionConfigs(frameIndex, :), cfg.links.center, 0.12);

    drawnow;
end

if ~cfg.visual.showAnimation
    close(fig);
end

if nargin >= 5
    fprintf("Final waypoint position error %.4f m, center-axis max angle %.2f deg.\n", ...
        ikResult.metrics(end).positionError, ikResult.metrics(end).maxCenterAxisAngleDeg);
end
end

function cameraState = captureCameraState(ax)
cameraState = struct();
cameraState.CameraPosition = ax.CameraPosition;
cameraState.CameraTarget = ax.CameraTarget;
cameraState.CameraUpVector = ax.CameraUpVector;
cameraState.CameraViewAngle = ax.CameraViewAngle;
cameraState.Projection = ax.Projection;
end

function restoreCameraState(ax, cameraState)
ax.CameraPosition = cameraState.CameraPosition;
ax.CameraTarget = cameraState.CameraTarget;
ax.CameraUpVector = cameraState.CameraUpVector;
ax.CameraViewAngle = cameraState.CameraViewAngle;
ax.Projection = cameraState.Projection;
end

function drawWallPlane(ax, wall, cfg)
patch(ax, ...
    "Faces", [1 2 3 4], ...
    "Vertices", wall.vertices, ...
    "FaceColor", [0.2 0.4 0.9], ...
    "FaceAlpha", cfg.visual.wallAlpha, ...
    "EdgeColor", [0.1 0.2 0.8], ...
    "LineWidth", 1.5);
end

function drawTargetPath(ax, trajectory)
hold(ax, "on");
plot3(ax, trajectory.targetPositions(:, 1), ...
    trajectory.targetPositions(:, 2), ...
    trajectory.targetPositions(:, 3), ...
    "m.-", "LineWidth", 2, "MarkerSize", 15);
hold(ax, "off");
end

function drawShovelSurface(ax, robot, config, cfg)
pointLinks = [cfg.links.forward, cfg.links.left, cfg.links.right];
points = zeros(3, 3);
for pointIndex = 1:3
    tform = getTransform(robot, config, pointLinks(pointIndex));
    points(pointIndex, :) = tform(1:3, 4).';
end

hold(ax, "on");
patch(ax, ...
    "Faces", [1 2 3], ...
    "Vertices", points, ...
    "FaceColor", "r", ...
    "FaceAlpha", cfg.visual.shovelPlaneAlpha, ...
    "EdgeColor", "r", ...
    "LineWidth", 2);
scatter3(ax, points(:, 1), points(:, 2), points(:, 3), 50, "r", "filled");
hold(ax, "off");
end

function drawFrameAxes(ax, robot, config, linkName, axisLength)
tform = getTransform(robot, config, linkName);
origin = tform(1:3, 4);
rotation = tform(1:3, 1:3);
axisVectors = rotation * eye(3) * axisLength;
colors = ["r", "g", "b"];
labels = ["X", "Y", "Z"];

hold(ax, "on");
for axisIndex = 1:3
    vector = axisVectors(:, axisIndex);
    quiver3(ax, origin(1), origin(2), origin(3), ...
        vector(1), vector(2), vector(3), ...
        "Color", colors(axisIndex), ...
        "LineWidth", 3, ...
        "MaxHeadSize", 0.8, ...
        "AutoScale", "off");
    text(ax, origin(1) + vector(1), origin(2) + vector(2), origin(3) + vector(3), ...
        " center " + labels(axisIndex), ...
        "Color", colors(axisIndex), ...
        "FontWeight", "bold", ...
        "FontSize", 11);
end
hold(ax, "off");
end
