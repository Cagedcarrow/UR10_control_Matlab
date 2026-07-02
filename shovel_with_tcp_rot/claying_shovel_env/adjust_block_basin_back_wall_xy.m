function robot = adjust_block_basin_back_wall_xy()
%ADJUST_BLOCK_BASIN_BACK_WALL_XY Preview and save block_basin_back_wall XY position.

scriptDir = fileparts(mfilename("fullpath"));
urdfPath = fullfile(scriptDir, "claying_shovel_env.urdf");
meshDir = fullfile(scriptDir, "meshes");
tempUrdfPath = fullfile(tempdir, "claying_shovel_env_back_wall_xy_preview.urdf");
targetJointName = "basin_back_joint";

if ~isfile(urdfPath)
    error("URDF file not found: %s", urdfPath);
end

if ~isfolder(meshDir)
    error("Mesh directory not found: %s", meshDir);
end

[wallXyz, wallRpy] = readJointOrigin(urdfPath, targetJointName);

state = struct();
state.urdfPath = urdfPath;
state.meshDir = meshDir;
state.tempUrdfPath = tempUrdfPath;
state.targetJointName = targetJointName;
state.wallXyz = wallXyz;
state.wallRpy = wallRpy;
state.previewXyz = wallXyz;
state.cameraInitialized = false;
state.cameraState = struct();
state.robot = [];
state.config = [];
state.ui = struct();

fig = figure("Name", "block_basin_back_wall XY adjuster", "Color", "w");
state.ui.ax = axes("Parent", fig, "Position", [0.08 0.28 0.88 0.66]);

uicontrol( ...
    fig, ...
    "Style", "text", ...
    "Units", "normalized", ...
    "Position", [0.10 0.19 0.22 0.035], ...
    "BackgroundColor", "w", ...
    "HorizontalAlignment", "left", ...
    "String", "X relative to block (m)");

state.ui.xEdit = uicontrol( ...
    fig, ...
    "Style", "edit", ...
    "Units", "normalized", ...
    "Position", [0.10 0.145 0.22 0.045], ...
    "BackgroundColor", "w", ...
    "HorizontalAlignment", "left", ...
    "String", sprintf("%.6f", state.previewXyz(1)), ...
    "Callback", @onApplyPreview);

uicontrol( ...
    fig, ...
    "Style", "text", ...
    "Units", "normalized", ...
    "Position", [0.34 0.19 0.22 0.035], ...
    "BackgroundColor", "w", ...
    "HorizontalAlignment", "left", ...
    "String", "Y relative to block (m)");

state.ui.yEdit = uicontrol( ...
    fig, ...
    "Style", "edit", ...
    "Units", "normalized", ...
    "Position", [0.34 0.145 0.22 0.045], ...
    "BackgroundColor", "w", ...
    "HorizontalAlignment", "left", ...
    "String", sprintf("%.6f", state.previewXyz(2)), ...
    "Callback", @onApplyPreview);

state.ui.applyButton = uicontrol( ...
    fig, ...
    "Style", "pushbutton", ...
    "Units", "normalized", ...
    "Position", [0.58 0.145 0.16 0.045], ...
    "String", "Preview XY", ...
    "Callback", @onApplyPreview);

state.ui.saveButton = uicontrol( ...
    fig, ...
    "Style", "pushbutton", ...
    "Units", "normalized", ...
    "Position", [0.10 0.075 0.30 0.045], ...
    "String", "Write current XY to URDF", ...
    "Callback", @onWriteBack);

state.ui.valueLabel = uicontrol( ...
    fig, ...
    "Style", "text", ...
    "Units", "normalized", ...
    "Position", [0.42 0.075 0.52 0.045], ...
    "BackgroundColor", "w", ...
    "HorizontalAlignment", "left", ...
    "String", "");

state.ui.statusLabel = uicontrol( ...
    fig, ...
    "Style", "text", ...
    "Units", "normalized", ...
    "Position", [0.10 0.025 0.84 0.04], ...
    "BackgroundColor", "w", ...
    "HorizontalAlignment", "left", ...
    "String", "");

refreshPreview();
robot = state.robot;

    function onApplyPreview(~, ~)
        xValue = str2double(strtrim(state.ui.xEdit.String));
        yValue = str2double(strtrim(state.ui.yEdit.String));
        if ~isfinite(xValue) || ~isfinite(yValue)
            state.ui.xEdit.String = sprintf("%.6f", state.previewXyz(1));
            state.ui.yEdit.String = sprintf("%.6f", state.previewXyz(2));
            setStatus("Invalid XY value. Enter numeric meter values.");
            return;
        end

        state.previewXyz(1:2) = [xValue, yValue];
        state.ui.xEdit.String = sprintf("%.6f", xValue);
        state.ui.yEdit.String = sprintf("%.6f", yValue);
        refreshPreview();
    end

    function onWriteBack(~, ~)
        rawUrdf = fileread(state.urdfPath);
        updatedUrdf = replaceJointOrigin(rawUrdf, state.targetJointName, ...
            state.previewXyz, state.wallRpy);
        writeTextFile(state.urdfPath, updatedUrdf);
        state.wallXyz = state.previewXyz;
        refreshPreview();
        setStatus(sprintf("Saved %s xyz = [%.6f %.6f %.6f].", ...
            state.targetJointName, state.wallXyz(1), state.wallXyz(2), state.wallXyz(3)));
    end

    function refreshPreview()
        rawUrdf = fileread(state.urdfPath);
        previewUrdf = replaceJointOrigin(rawUrdf, state.targetJointName, ...
            state.previewXyz, state.wallRpy);
        writeTextFile(state.tempUrdfPath, previewUrdf);

        state.robot = importrobot(state.tempUrdfPath, ...
            "DataFormat", "row", ...
            "MeshPath", state.meshDir);
        state.config = initialSceneConfiguration(state.robot);
        drawScene();
        state.ui.valueLabel.String = sprintf("Preview xyz = [%.6f %.6f %.6f] m", ...
            state.previewXyz(1), state.previewXyz(2), state.previewXyz(3));
        setStatus("Preview only. Click write button to save into claying_shovel_env.urdf.");
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
        title(state.ui.ax, "block\_basin\_back\_wall XY preview");
        drawBackWallMarker();
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

    function drawBackWallMarker()
        if ~all(ismember(["block_basin_block", "block_basin_back_wall"], string(state.robot.BodyNames)))
            return;
        end

        blockTform = getTransform(state.robot, state.config, "block_basin_block");
        wallTform = getTransform(state.robot, state.config, "block_basin_back_wall");
        blockOrigin = blockTform(1:3, 4);
        wallOrigin = wallTform(1:3, 4);

        hold(state.ui.ax, "on");
        plot3(state.ui.ax, ...
            [blockOrigin(1), wallOrigin(1)], ...
            [blockOrigin(2), wallOrigin(2)], ...
            [blockOrigin(3), wallOrigin(3)], ...
            "m-", "LineWidth", 2);
        scatter3(state.ui.ax, wallOrigin(1), wallOrigin(2), wallOrigin(3), ...
            70, "m", "filled");
        text(state.ui.ax, wallOrigin(1), wallOrigin(2), wallOrigin(3), ...
            " block\_basin\_back\_wall", ...
            "Color", "m", ...
            "FontWeight", "bold", ...
            "FontSize", 11);
        hold(state.ui.ax, "off");
    end

    function setStatus(text)
        state.ui.statusLabel.String = char(text);
    end
end

function [xyz, rpy] = readJointOrigin(urdfPath, jointName)
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

function updatedUrdf = replaceJointOrigin(urdfText, jointName, xyz, rpy)
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
