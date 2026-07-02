function outputPaths = plot_mud_smearing_joint_angles(result)
%PLOT_MUD_SMEARING_JOINT_ANGLES Save all movable joint angles in one figure.

scriptDir = fileparts(fileparts(mfilename("fullpath")));
figuresDir = fullfile(scriptDir, "figures");
if ~isfolder(figuresDir)
    mkdir(figuresDir);
end

jointNames = list_movable_joint_names(result.robot);
jointAnglesDeg = rad2deg(result.motionConfigs);
frameIndex = 1:size(jointAnglesDeg, 1);

fig = figure("Name", "mud smearing joint angles", "Color", "w", "Visible", "off");
ax = axes("Parent", fig);
plot(ax, frameIndex, jointAnglesDeg, "LineWidth", 1.8);
grid(ax, "on");
xlabel(ax, "Frame index");
ylabel(ax, "Joint angle (deg)");
title(ax, "UR10 joints and shovel motor angle during mud smearing");
legend(ax, readableJointLabels(jointNames), ...
    "Interpreter", "none", ...
    "Location", "eastoutside");

pngPath = fullfile(figuresDir, "mud_smearing_joint_angles.png");
figPath = fullfile(figuresDir, "mud_smearing_joint_angles.fig");
exportgraphics(fig, pngPath, "Resolution", 200);
savefig(fig, figPath);
close(fig);

outputPaths = struct();
outputPaths.png = pngPath;
outputPaths.fig = figPath;
end

function labels = readableJointLabels(jointNames)
labels = strings(size(jointNames));
for idx = 1:numel(jointNames)
    switch jointNames(idx)
        case "ur10_shoulder_pan"
            labels(idx) = "UR10 joint 1 shoulder pan";
        case "ur10_shoulder_lift"
            labels(idx) = "UR10 joint 2 shoulder lift";
        case "ur10_elbow"
            labels(idx) = "UR10 joint 3 elbow";
        case "ur10_wrist_1"
            labels(idx) = "UR10 joint 4 wrist 1";
        case "ur10_wrist_2"
            labels(idx) = "UR10 joint 5 wrist 2";
        case "ur10_wrist_3"
            labels(idx) = "UR10 joint 6 wrist 3";
        case "base_tail_to_shovel_base"
            labels(idx) = "Shovel motor base_tail_to_shovel_base";
        otherwise
            labels(idx) = jointNames(idx);
    end
end
end
