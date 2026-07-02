function robot = adjust_shovel_mount_z_rotation()
%ADJUST_SHOVEL_MOUNT_Z_ROTATION Preview and save shovel mount yaw in the scene URDF.

scriptDir = fileparts(mfilename("fullpath"));
urdfPath = fullfile(scriptDir, "claying_shovel_env.urdf");
meshDir = fullfile(scriptDir, "meshes");
tempUrdfPath = fullfile(tempdir, "claying_shovel_env_mount_preview.urdf");
targetJointName = "ur10_wrist_3_to_shovel_rotate_base";

if ~isfile(urdfPath)
    error("URDF file not found: %s", urdfPath);
end

if ~isfolder(meshDir)
    error("Mesh directory not found: %s", meshDir);
end

[mountXyz, mountRpy] = readMountOrigin(urdfPath, targetJointName);
state = struct();
state.urdfPath = urdfPath;
state.meshDir = meshDir;
state.tempUrdfPath = tempUrdfPath;
state.targetJointName = targetJointName;
state.mountXyz = mountXyz;
state.mountRpy = mountRpy;
state.localZDeltaRad = 0;
state.cameraInitialized = false;
state.cameraState = struct();

state.robot = [];
state.config = [];
state.ui = struct();

fig = figure("Name", "shovel mount Z rotation adjuster", "Color", "w");
state.ui.ax = axes("Parent", fig, "Position", [0.08 0.26 0.88 0.68]);

uicontrol( ...
    fig, ...
    "Style", "text", ...
    "Units", "normalized", ...
    "Position", [0.10 0.16 0.34 0.04], ...
    "BackgroundColor", "w", ...
    "HorizontalAlignment", "left", ...
    "String", "base_link local Z delta (deg)");

state.ui.angleEdit = uicontrol( ...
    fig, ...
    "Style", "edit", ...
    "Units", "normalized", ...
    "Position", [0.10 0.09 0.20 0.05], ...
    "BackgroundColor", "w", ...
    "HorizontalAlignment", "left", ...
    "String", sprintf("%.2f", rad2deg(state.localZDeltaRad)), ...
    "Callback", @onLocalZEditChanged);

state.ui.applyButton = uicontrol( ...
    fig, ...
    "Style", "pushbutton", ...
    "Units", "normalized", ...
    "Position", [0.32 0.09 0.16 0.05], ...
    "String", "Apply angle", ...
    "Callback", @onLocalZEditChanged);

state.ui.valueLabel = uicontrol( ...
    fig, ...
    "Style", "text", ...
    "Units", "normalized", ...
    "Position", [0.50 0.09 0.44 0.05], ...
    "BackgroundColor", "w", ...
    "HorizontalAlignment", "left", ...
    "String", "");

state.ui.saveButton = uicontrol( ...
    fig, ...
    "Style", "pushbutton", ...
    "Units", "normalized", ...
    "Position", [0.10 0.03 0.26 0.045], ...
    "String", "Write current mount to URDF", ...
    "Callback", @onWriteBack);

state.ui.statusLabel = uicontrol( ...
    fig, ...
    "Style", "text", ...
    "Units", "normalized", ...
    "Position", [0.38 0.03 0.56 0.045], ...
    "BackgroundColor", "w", ...
    "HorizontalAlignment", "left", ...
    "String", "");

refreshPreview();
robot = state.robot;

    function onLocalZEditChanged(~, ~)
        angleDeg = str2double(strtrim(state.ui.angleEdit.String));
        if ~isfinite(angleDeg)
            state.ui.angleEdit.String = sprintf("%.2f", rad2deg(state.localZDeltaRad));
            setStatus("Invalid angle. Enter a numeric degree value.");
            return;
        end

        state.localZDeltaRad = deg2rad(angleDeg);
        state.ui.angleEdit.String = sprintf("%.2f", angleDeg);
        refreshPreview();
    end

    function onWriteBack(~, ~)
        savedRpy = composeLocalZRotation(state.mountRpy, state.localZDeltaRad);
        rawUrdf = fileread(state.urdfPath);
        updatedUrdf = replaceMountOrigin(rawUrdf, state.targetJointName, ...
            state.mountXyz, savedRpy);
        writeTextFile(state.urdfPath, updatedUrdf);
        state.mountRpy = savedRpy;
        state.localZDeltaRad = 0;
        state.ui.angleEdit.String = "0.00";
        refreshPreview();
        setStatus(sprintf("Saved local Z delta. New rpy = [%.6f %.6f %.6f].", savedRpy(1), savedRpy(2), savedRpy(3)));
    end

    function refreshPreview()
        previewRpy = composeLocalZRotation(state.mountRpy, state.localZDeltaRad);
        rawUrdf = fileread(state.urdfPath);
        previewUrdf = replaceMountOrigin(rawUrdf, state.targetJointName, ...
            state.mountXyz, previewRpy);
        writeTextFile(state.tempUrdfPath, previewUrdf);

        state.robot = importrobot(state.tempUrdfPath, ...
            "DataFormat", "row", ...
            "MeshPath", state.meshDir);
        state.config = initialSceneConfiguration(state.robot);
        drawScene();
        state.ui.valueLabel.String = sprintf("%.2f deg", rad2deg(state.localZDeltaRad));
        setStatus(sprintf("Preview local Z delta %.3f deg. Click write button to save.", rad2deg(state.localZDeltaRad)));
    end

    function drawScene()
        rememberCameraState();
        show(state.robot, state.config, ...
            "Parent", state.ui.ax, ...
            "Visuals", "on", ...
            "Collisions", "off", ...
            "Frames", "on", ...
            "PreservePlot", false);
        axis(state.ui.ax, "equal");
        grid(state.ui.ax, "on");
        restoreCameraState();
        title(state.ui.ax, "claying\_shovel\_env mount yaw preview");
        drawBaseLinkAxes();
        drawnow limitrate;
    end

    function rememberCameraState()
        if ~state.cameraInitialized || ~isgraphics(state.ui.ax)
            return;
        end

        state.cameraState.CameraPosition = state.ui.ax.CameraPosition;
        state.cameraState.CameraTarget = state.ui.ax.CameraTarget;
        state.cameraState.CameraUpVector = state.ui.ax.CameraUpVector;
        state.cameraState.CameraViewAngle = state.ui.ax.CameraViewAngle;
        state.cameraState.Projection = state.ui.ax.Projection;
    end

    function restoreCameraState()
        if state.cameraInitialized
            state.ui.ax.CameraPosition = state.cameraState.CameraPosition;
            state.ui.ax.CameraTarget = state.cameraState.CameraTarget;
            state.ui.ax.CameraUpVector = state.cameraState.CameraUpVector;
            state.ui.ax.CameraViewAngle = state.cameraState.CameraViewAngle;
            state.ui.ax.Projection = state.cameraState.Projection;
            return;
        end

        view(state.ui.ax, 135, 20);
        state.cameraInitialized = true;
        rememberCameraState();
    end

    function drawBaseLinkAxes()
        if ~any(strcmp(state.robot.BodyNames, "base_link"))
            return;
        end

        tform = getTransform(state.robot, state.config, "base_link");
        origin = tform(1:3, 4);
        rotation = tform(1:3, 1:3);
        axisLength = 0.25;
        axesVectors = rotation * eye(3) * axisLength;
        labels = ["X", "Y", "Z"];
        colors = ["r", "g", "b"];

        hold(state.ui.ax, "on");
        for axisIndex = 1:3
            vector = axesVectors(:, axisIndex);
            quiver3(state.ui.ax, origin(1), origin(2), origin(3), ...
                vector(1), vector(2), vector(3), ...
                "Color", colors(axisIndex), ...
                "LineWidth", 4, ...
                "MaxHeadSize", 1.0, ...
                "AutoScale", "off");
            text(state.ui.ax, ...
                origin(1) + vector(1), ...
                origin(2) + vector(2), ...
                origin(3) + vector(3), ...
                " base\_link " + labels(axisIndex), ...
                "Color", colors(axisIndex), ...
                "FontWeight", "bold", ...
                "FontSize", 12);
        end
        hold(state.ui.ax, "off");
    end

    function setStatus(text)
        state.ui.statusLabel.String = char(text);
    end
end

function rpy = composeLocalZRotation(baseRpy, localZDeltaRad)
baseRotation = rpyToRotm(baseRpy);
deltaRotation = rotz3(localZDeltaRad);
rpy = rotmToRpy(baseRotation * deltaRotation);
end

function rotation = rpyToRotm(rpy)
roll = rpy(1);
pitch = rpy(2);
yaw = rpy(3);
rotation = rotz3(yaw) * roty3(pitch) * rotx3(roll);
end

function rpy = rotmToRpy(rotation)
pitch = atan2(-rotation(3, 1), hypot(rotation(1, 1), rotation(2, 1)));

if abs(cos(pitch)) > 1e-9
    roll = atan2(rotation(3, 2), rotation(3, 3));
    yaw = atan2(rotation(2, 1), rotation(1, 1));
else
    roll = atan2(-rotation(2, 3), rotation(2, 2));
    yaw = 0;
end

rpy = [roll, pitch, yaw];
end

function rotation = rotx3(angle)
rotation = [ ...
    1, 0, 0; ...
    0, cos(angle), -sin(angle); ...
    0, sin(angle), cos(angle)];
end

function rotation = roty3(angle)
rotation = [ ...
    cos(angle), 0, sin(angle); ...
    0, 1, 0; ...
    -sin(angle), 0, cos(angle)];
end

function rotation = rotz3(angle)
rotation = [ ...
    cos(angle), -sin(angle), 0; ...
    sin(angle), cos(angle), 0; ...
    0, 0, 1];
end

function [xyz, rpy] = readMountOrigin(urdfPath, jointName)
urdfText = fileread(urdfPath);
jointPattern = sprintf('<joint\\s+name="%s"\\s+type="fixed">[\\s\\S]*?<origin\\s+xyz="([^"]+)"\\s+rpy="([^"]+)"\\s*/>[\\s\\S]*?</joint>', jointName);
tokens = regexp(urdfText, jointPattern, "tokens", "once");
if isempty(tokens)
    error("Target joint origin not found: %s", jointName);
end

xyz = sscanf(tokens{1}, "%f").';
rpy = sscanf(tokens{2}, "%f").';
if numel(xyz) ~= 3 || numel(rpy) ~= 3
    error("Invalid origin on target joint: %s", jointName);
end
end

function updatedUrdf = replaceMountOrigin(urdfText, jointName, xyz, rpy)
xyzText = sprintf("%.12g %.12g %.12g", xyz(1), xyz(2), xyz(3));
rpyText = sprintf("%.12g %.12g %.12g", rpy(1), rpy(2), rpy(3));
jointPattern = sprintf('(<joint\\s+name="%s"\\s+type="fixed">[\\s\\S]*?<origin\\s+xyz=")([^"]+)("\\s+rpy=")([^"]+)("\\s*/>[\\s\\S]*?</joint>)', jointName);
tokens = regexp(urdfText, jointPattern, "tokens", "once");
[matchStart, matchEnd] = regexp(urdfText, jointPattern, "start", "end", "once");
if isempty(tokens)
    error("Target joint origin pattern not found: %s", jointName);
end
replacement = [tokens{1}, xyzText, tokens{3}, rpyText, tokens{5}];
updatedUrdf = [urdfText(1:matchStart - 1), replacement, urdfText(matchEnd + 1:end)];
end

function writeTextFile(filePath, text)
fid = fopen(filePath, "w", "n", "UTF-8");
if fid < 0
    error("Unable to write file: %s", filePath);
end
cleanupObj = onCleanup(@() fclose(fid)); %#ok<NASGU>
fwrite(fid, char(text), "char");
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
