function robot = view_ur10_urdf(modelName)
%VIEW_UR10_URDF Load and visualize the UR10 + rotating shovel URDF.

if nargin < 1 || strlength(string(modelName)) == 0
    modelName = "ur10_with_rotate_shovel";
end

scriptDir = fileparts(mfilename("fullpath"));
modelName = string(modelName);
if modelName == "ur10_only" || modelName == "ur10_with_rotate_shovel"
    urdfFile = modelName + ".urdf";
else
    urdfFile = modelName;
end
urdfPath = fullfile(scriptDir, urdfFile);
meshDir = fullfile(scriptDir, "meshes");

if ~isfile(urdfPath)
    error("URDF file not found: %s", urdfPath);
end

if ~isfolder(meshDir)
    error("Mesh directory not found: %s", meshDir);
end

fprintf("Loading URDF: %s\n", urdfPath);
robot = importrobot(urdfPath, "DataFormat", "row", "MeshPath", meshDir);

config = homeConfiguration(robot);
[jointNames, jointLimits] = getMovableJoints(robot);

fig = figure("Name", "UR10 rotating shovel URDF viewer", "Color", "w");
ax = axes("Parent", fig, "Position", [0.08 0.30 0.88 0.65]);
sliderPanel = uipanel( ...
    "Parent", fig, ...
    "Units", "normalized", ...
    "Position", [0.04 0.03 0.92 0.22], ...
    "BorderType", "none", ...
    "BackgroundColor", "w");

drawRobot();
buildJointSliders();

    function buildJointSliders()
        jointCount = numel(jointNames);
        rowHeight = 1 / max(jointCount, 1);

        for jointIndex = 1:jointCount
            y = 1 - jointIndex * rowHeight + 0.01;
            limits = jointLimits(jointIndex, :);

            uicontrol( ...
                sliderPanel, ...
                "Style", "text", ...
                "Units", "normalized", ...
                "Position", [0.00 y 0.24 rowHeight * 0.75], ...
                "BackgroundColor", "w", ...
                "HorizontalAlignment", "left", ...
                "String", jointNames(jointIndex));

            valueLabel = uicontrol( ...
                sliderPanel, ...
                "Style", "text", ...
                "Units", "normalized", ...
                "Position", [0.86 y 0.13 rowHeight * 0.75], ...
                "BackgroundColor", "w", ...
                "HorizontalAlignment", "left", ...
                "String", sprintf("%.1f deg", rad2deg(config(jointIndex))));

            uicontrol( ...
                sliderPanel, ...
                "Style", "slider", ...
                "Units", "normalized", ...
                "Position", [0.25 y + 0.01 0.59 rowHeight * 0.55], ...
                "Min", limits(1), ...
                "Max", limits(2), ...
                "Value", config(jointIndex), ...
                "Callback", @(src, ~) updateJoint(jointIndex, src.Value, valueLabel));
        end
    end

    function updateJoint(jointIndex, value, valueLabel)
        config(jointIndex) = value;
        valueLabel.String = sprintf("%.1f deg", rad2deg(value));
        drawRobot();
    end

    function drawRobot()
        show(robot, config, ...
            "Parent", ax, ...
            "Visuals", "on", ...
            "Collisions", "off", ...
            "Frames", "on", ...
            "PreservePlot", false);
        axis(ax, "equal");
        grid(ax, "on");
        view(ax, 135, 20);
        title(ax, strrep(erase(urdfFile, ".urdf"), "_", "\_"));
        drawnow limitrate;
    end
end

function [jointNames, jointLimits] = getMovableJoints(robot)
jointNames = strings(1, 0);
jointLimits = zeros(0, 2);

for bodyIndex = 1:robot.NumBodies
    joint = robot.Bodies{bodyIndex}.Joint;
    if strcmp(joint.Type, "fixed")
        continue;
    end

    limits = joint.PositionLimits;
    if numel(limits) ~= 2 || any(~isfinite(limits)) || limits(1) == limits(2)
        limits = [-pi pi];
    end

    jointNames(end + 1) = string(joint.Name); %#ok<AGROW>
    jointLimits(end + 1, :) = limits; %#ok<AGROW>
end
end
