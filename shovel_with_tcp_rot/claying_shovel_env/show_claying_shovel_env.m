function robot = show_claying_shovel_env()
%SHOW_CLAYING_SHOVEL_ENV Load and display the migrated claying shovel scene.

scriptDir = fileparts(mfilename("fullpath"));
urdfPath = fullfile(scriptDir, "claying_shovel_env.urdf");
meshDir = fullfile(scriptDir, "meshes");

if ~isfile(urdfPath)
    error("URDF file not found: %s", urdfPath);
end

if ~isfolder(meshDir)
    error("Mesh directory not found: %s", meshDir);
end

fprintf("Loading URDF: %s\n", urdfPath);
robot = importrobot(urdfPath, "DataFormat", "row", "MeshPath", meshDir);
config = initialSceneConfiguration(robot);
[shovelJointIndex, shovelJointLimits] = findMovableJoint(robot, "base_tail_to_shovel_base");

fig = figure("Name", "claying shovel environment", "Color", "w");
ax = axes("Parent", fig, "Position", [0.08 0.24 0.88 0.70]);

uicontrol( ...
    fig, ...
    "Style", "text", ...
    "Units", "normalized", ...
    "Position", [0.10 0.13 0.28 0.04], ...
    "BackgroundColor", "w", ...
    "HorizontalAlignment", "left", ...
    "String", "shovel rotate angle (deg)");

valueLabel = uicontrol( ...
    fig, ...
    "Style", "text", ...
    "Units", "normalized", ...
    "Position", [0.78 0.06 0.15 0.05], ...
    "BackgroundColor", "w", ...
    "HorizontalAlignment", "left", ...
    "String", sprintf("%.1f deg", rad2deg(config(shovelJointIndex))));

sliderHandle = uicontrol( ...
    fig, ...
    "Style", "slider", ...
    "Units", "normalized", ...
    "Position", [0.10 0.06 0.66 0.05], ...
    "Min", shovelJointLimits(1), ...
    "Max", shovelJointLimits(2), ...
    "Value", config(shovelJointIndex), ...
    "Callback", @updateShovelRotation);

drawScene();

    function updateShovelRotation(src, ~)
        config(shovelJointIndex) = src.Value;
        valueLabel.String = sprintf("%.1f deg", rad2deg(src.Value));
        drawScene();
    end

    function drawScene()
        show(robot, config, ...
            "Parent", ax, ...
            "Visuals", "on", ...
            "Collisions", "off", ...
            "Frames", "on", ...
            "PreservePlot", false);
        axis(ax, "equal");
        grid(ax, "on");
        view(ax, 135, 20);
        title(ax, "claying\_shovel\_env");
        drawShovelSurfacePlane();
        drawnow limitrate;
    end

    function drawShovelSurfacePlane()
        pointLinks = ["shovel_forward_link", "shovel_left_link", "shovel_right_link"];
        if ~all(ismember(pointLinks, string(robot.BodyNames)))
            warning("Shovel surface point links were not all found. Red plane was not drawn.");
            return;
        end

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
            "FaceAlpha", 0.35, ...
            "EdgeColor", "r", ...
            "LineWidth", 2);
        scatter3(ax, points(:, 1), points(:, 2), points(:, 3), ...
            50, "r", "filled");
        hold(ax, "off");
    end
end

function config = initialSceneConfiguration(robot)
config = homeConfiguration(robot);
jointValues = struct( ...
    "ur10_shoulder_pan", 2.0474984645843506, ...
    "ur10_shoulder_lift", 0.21928656101226807, ...
    "ur10_elbow", -1.9548214117633265, ...
    "ur10_wrist_1", -0.35923797289003545, ...
    "ur10_wrist_2", 2.0502676963806152, ...
    "ur10_wrist_3", 1.0330829620361328, ...
    "base_tail_to_shovel_base", 0);

movableJointIndex = 0;
for bodyIndex = 1:robot.NumBodies
    joint = robot.Bodies{bodyIndex}.Joint;
    if strcmp(joint.Type, "fixed")
        continue;
    end

    movableJointIndex = movableJointIndex + 1;
    jointName = char(joint.Name);
    if isfield(jointValues, jointName)
        config(movableJointIndex) = jointValues.(jointName);
    end
end
end

function [jointIndex, jointLimits] = findMovableJoint(robot, jointName)
jointIndex = [];
jointLimits = [-pi pi];
movableJointIndex = 0;

for bodyIndex = 1:robot.NumBodies
    joint = robot.Bodies{bodyIndex}.Joint;
    if strcmp(joint.Type, "fixed")
        continue;
    end

    movableJointIndex = movableJointIndex + 1;
    if string(joint.Name) == jointName
        jointIndex = movableJointIndex;
        limits = joint.PositionLimits;
        if numel(limits) == 2 && all(isfinite(limits)) && limits(1) ~= limits(2)
            jointLimits = limits;
        end
        return;
    end
end

error("Joint %s was not found as a movable joint.", jointName);
end
