function adjust_block_with_basin_pose_gui()
%ADJUST_BLOCK_WITH_BASIN_POSE_GUI Visual pose editor for block_with_basin in the full scene.
%
% This GUI edits the joint:
%   base_jizuo_to_block_with_basin_frame
% in:
%   environmental_model/assembly_with_block_with_basin.urdf
%
% Outputs:
%   1) Saves pose parameters to YAML
%   2) Replaces the corresponding URDF joint origin in-place

paths = buildPaths();
ensureRequiredFiles(paths);

state = struct();
state.paths = paths;
state.pose = loadInitialPose(paths);
state.initialJointPosition = readInitialJointPosition(paths.sceneUrdf);
state.ui = struct();
state.robot = [];
state.tempUrdfPath = fullfile(tempdir, 'assembly_with_block_with_basin_pose_preview.urdf');

buildUi();
refreshPreview();

    function buildUi()
        fig = uifigure( ...
            'Name', '场景位置调整 GUI', ...
            'Position', [80 60 1500 860], ...
            'Color', [0.94 0.94 0.94]);

        mainGrid = uigridlayout(fig, [3 2], ...
            'RowHeight', {44, '1x', 34}, ...
            'ColumnWidth', {360, '1x'}, ...
            'Padding', [8 8 8 6], ...
            'RowSpacing', 6, ...
            'ColumnSpacing', 8);

        toolbar = uigridlayout(mainGrid, [1 7], ...
            'ColumnWidth', {110, 110, 120, 120, 120, 120, '1x'}, ...
            'RowHeight', {34}, ...
            'Padding', [0 0 0 0]);
        toolbar.Layout.Row = 1;
        toolbar.Layout.Column = [1 2];

        uibutton(toolbar, 'push', ...
            'Text', '从 URDF 重载', ...
            'ButtonPushedFcn', @(~,~) onReloadFromUrdf());
        uibutton(toolbar, 'push', ...
            'Text', '从 YAML 重载', ...
            'ButtonPushedFcn', @(~,~) onReloadFromYaml());
        uibutton(toolbar, 'push', ...
            'Text', '保存 YAML', ...
            'ButtonPushedFcn', @(~,~) onSaveYaml());
        uibutton(toolbar, 'push', ...
            'Text', '写回 URDF', ...
            'ButtonPushedFcn', @(~,~) onApplyUrdf());
        uibutton(toolbar, 'push', ...
            'Text', '保存 YAML + URDF', ...
            'ButtonPushedFcn', @(~,~) onSaveAll());
        uibutton(toolbar, 'push', ...
            'Text', '重置为零位', ...
            'ButtonPushedFcn', @(~,~) onResetPose());

        leftPanel = uipanel(mainGrid, 'Title', '相对基座位姿参数');
        leftPanel.Layout.Row = 2;
        leftPanel.Layout.Column = 1;

        panelGrid = uigridlayout(leftPanel, [10 3], ...
            'RowHeight', {28, 28, 28, 28, 28, 28, 16, 28, 60, '1x'}, ...
            'ColumnWidth', {95, '1x', 90}, ...
            'Padding', [8 8 8 8], ...
            'RowSpacing', 6, ...
            'ColumnSpacing', 8);

        addPoseControl(panelGrid, 1, 'X [m]', 'x', -3.0, 3.0);
        addPoseControl(panelGrid, 2, 'Y [m]', 'y', -3.0, 3.0);
        addPoseControl(panelGrid, 3, 'Z [m]', 'z', -1.0, 3.0);
        addPoseControl(panelGrid, 4, 'Roll [deg]', 'rollDeg', -180, 180);
        addPoseControl(panelGrid, 5, 'Pitch [deg]', 'pitchDeg', -180, 180);
        addPoseControl(panelGrid, 6, 'Yaw [deg]', 'yawDeg', -180, 180);

        helpLabel = uilabel(panelGrid, ...
            'Text', sprintf(['说明:\n' ...
            '1. 这里调整的是 block_with_basin 相对机械臂基座 base_jizuo 的整体位姿。\n' ...
            '2. 预览实时刷新，但正式 URDF 只有点击“写回 URDF”或“保存 YAML + URDF”才会改。']), ...
            'WordWrap', 'on', ...
            'HorizontalAlignment', 'left');
        helpLabel.Layout.Row = 9;
        helpLabel.Layout.Column = [1 3];

        state.ui.lblPaths = uilabel(panelGrid, ...
            'Text', sprintf('URDF: %s\nYAML: %s', paths.sceneUrdf, paths.poseYaml), ...
            'WordWrap', 'on', ...
            'HorizontalAlignment', 'left');
        state.ui.lblPaths.Layout.Row = 10;
        state.ui.lblPaths.Layout.Column = [1 3];

        axesPanel = uipanel(mainGrid, 'Title', '场景预览');
        axesPanel.Layout.Row = 2;
        axesPanel.Layout.Column = 2;
        state.ui.axesPanel = axesPanel;

        statusGrid = uigridlayout(mainGrid, [1 2], ...
            'ColumnWidth', {'1x', 420}, ...
            'RowHeight', {24}, ...
            'Padding', [4 0 4 0]);
        statusGrid.Layout.Row = 3;
        statusGrid.Layout.Column = [1 2];

        state.ui.lblStatus = uilabel(statusGrid, ...
            'Text', '已加载。拖动参数可实时预览。', ...
            'HorizontalAlignment', 'left');
        state.ui.lblPose = uilabel(statusGrid, ...
            'Text', buildPoseSummary(state.pose), ...
            'HorizontalAlignment', 'right');
    end

    function addPoseControl(parent, row, labelText, fieldName, minVal, maxVal)
        label = uilabel(parent, 'Text', labelText, 'HorizontalAlignment', 'right');
        label.Layout.Row = row;
        label.Layout.Column = 1;

        slider = uislider(parent, ...
            'Limits', [minVal, maxVal], ...
            'Value', state.pose.(fieldName), ...
            'MajorTicks', [], ...
            'ValueChangingFcn', @(src, evt) onPoseChanging(fieldName, evt.Value), ...
            'ValueChangedFcn', @(src, evt) onPoseChanged(fieldName, evt.Value));
        slider.Layout.Row = row;
        slider.Layout.Column = 2;

        spinner = uieditfield(parent, 'numeric', ...
            'Limits', [minVal, maxVal], ...
            'Value', state.pose.(fieldName), ...
            'RoundFractionalValues', false, ...
            'ValueDisplayFormat', '%.6f', ...
            'ValueChangedFcn', @(src, evt) onPoseChanged(fieldName, src.Value));
        spinner.Layout.Row = row;
        spinner.Layout.Column = 3;

        state.ui.([fieldName 'Slider']) = slider;
        state.ui.([fieldName 'Edit']) = spinner;
    end

    function onPoseChanging(fieldName, value)
        state.pose.(fieldName) = value;
        syncControlPair(fieldName, value, 'slider');
        refreshPreview();
    end

    function onPoseChanged(fieldName, value)
        state.pose.(fieldName) = value;
        syncControlPair(fieldName, value, 'both');
        refreshPreview();
    end

    function syncControlPair(fieldName, value, source)
        if ~strcmp(source, 'slider')
            state.ui.([fieldName 'Slider']).Value = value;
        end
        if ~strcmp(source, 'edit')
            state.ui.([fieldName 'Edit']).Value = value;
        end
        state.ui.lblPose.Text = buildPoseSummary(state.pose);
    end

    function onReloadFromUrdf()
        state.pose = readPoseFromUrdf(paths.sceneUrdf);
        syncAllControls();
        refreshPreview();
        setStatus('已从 URDF 重新读取 block_with_basin 位姿。');
    end

    function onReloadFromYaml()
        if ~isfile(paths.poseYaml)
            uialert(ancestor(state.ui.lblStatus, 'figure'), ...
                sprintf('YAML 文件不存在:\n%s', paths.poseYaml), ...
                '读取失败');
            return;
        end
        state.pose = readPoseFromYaml(paths.poseYaml);
        syncAllControls();
        refreshPreview();
        setStatus('已从 YAML 重新读取位姿。');
    end

    function onSaveYaml()
        writePoseYaml(paths.poseYaml, state.pose, paths.sceneUrdf);
        setStatus(sprintf('YAML 已保存: %s', paths.poseYaml));
    end

    function onApplyUrdf()
        writePoseToUrdf(paths.sceneUrdf, state.pose);
        setStatus(sprintf('URDF 已写回: %s', paths.sceneUrdf));
    end

    function onSaveAll()
        writePoseYaml(paths.poseYaml, state.pose, paths.sceneUrdf);
        writePoseToUrdf(paths.sceneUrdf, state.pose);
        setStatus('YAML 和 URDF 都已更新。');
    end

    function onResetPose()
        state.pose = struct('x', 0, 'y', 0, 'z', 0, 'rollDeg', 0, 'pitchDeg', 0, 'yawDeg', 0);
        syncAllControls();
        refreshPreview();
        setStatus('已重置为零位。');
    end

    function syncAllControls()
        fields = {'x', 'y', 'z', 'rollDeg', 'pitchDeg', 'yawDeg'};
        for i = 1:numel(fields)
            fieldName = fields{i};
            state.ui.([fieldName 'Slider']).Value = state.pose.(fieldName);
            state.ui.([fieldName 'Edit']).Value = state.pose.(fieldName);
        end
        state.ui.lblPose.Text = buildPoseSummary(state.pose);
    end

    function refreshPreview()
        try
            previewText = buildPreviewUrdfText(paths.sceneUrdf, state.pose);
            writeTextFile(state.tempUrdfPath, previewText);
            robot = importrobot(state.tempUrdfPath, ...
                'DataFormat', 'row', ...
                'MeshPath', paths.meshDir);
            state.robot = robot;
            redrawAxes();
        catch ME
            setStatus(sprintf('预览失败: %s', ME.message));
        end
    end

    function redrawAxes()
        delete(findobj(state.ui.axesPanel, 'Type', 'axes'));
        ax = axes(state.ui.axesPanel); %#ok<LAXES>
        show(state.robot, ...
            state.initialJointPosition, ...
            'Visuals', 'on', ...
            'Collisions', 'off', ...
            'Frames', 'on', ...
            'Parent', ax);
        axis(ax, 'equal');
        grid(ax, 'on');
        view(ax, 135, 20);
        hold(ax, 'on');
        len = 0.35;
        quiver3(ax, 0, 0, 0, len, 0, 0, 'r', 'LineWidth', 2, 'MaxHeadSize', 0.6);
        quiver3(ax, 0, 0, 0, 0, len, 0, 'g', 'LineWidth', 2, 'MaxHeadSize', 0.6);
        quiver3(ax, 0, 0, 0, 0, 0, len, 'b', 'LineWidth', 2, 'MaxHeadSize', 0.6);
        hold(ax, 'off');
        title(ax, 'assembly\_with\_block\_with\_basin 预览');
    end

    function setStatus(text)
        state.ui.lblStatus.Text = text;
    end
end

function paths = buildPaths()
thisDir = fileparts(mfilename('fullpath'));
paths = struct();
paths.thisDir = thisDir;
paths.sceneUrdf = fullfile(thisDir, 'assembly_with_block_with_basin.urdf');
paths.poseYaml = fullfile(thisDir, 'block_with_basin_pose.yaml');
paths.meshDir = fullfile(thisDir, 'meshes');
end

function ensureRequiredFiles(paths)
requiredPaths = {paths.sceneUrdf, paths.meshDir};
for i = 1:numel(requiredPaths)
    thisPath = requiredPaths{i};
    if ~exist(thisPath, 'file') && ~exist(thisPath, 'dir')
        error('adjust_block_with_basin_pose_gui:MissingPath', ...
            'Required path not found: %s', thisPath);
    end
end
end

function pose = loadInitialPose(paths)
if isfile(paths.poseYaml)
    pose = readPoseFromYaml(paths.poseYaml);
else
    pose = readPoseFromUrdf(paths.sceneUrdf);
    writePoseYaml(paths.poseYaml, pose, paths.sceneUrdf);
end
end

function pose = readPoseFromYaml(yamlPath)
yamlText = fileread(yamlPath);
pose = struct( ...
    'x', readYamlScalar(yamlText, 'x'), ...
    'y', readYamlScalar(yamlText, 'y'), ...
    'z', readYamlScalar(yamlText, 'z'), ...
    'rollDeg', readYamlScalar(yamlText, 'roll_deg'), ...
    'pitchDeg', readYamlScalar(yamlText, 'pitch_deg'), ...
    'yawDeg', readYamlScalar(yamlText, 'yaw_deg'));
end

function value = readYamlScalar(yamlText, key)
tokens = regexp(yamlText, ['(?m)^\s*' regexptranslate('escape', key) '\s*:\s*([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)\s*$'], 'tokens', 'once');
if isempty(tokens)
    error('adjust_block_with_basin_pose_gui:YamlParseFailed', ...
        'Key not found in YAML: %s', key);
end
value = str2double(tokens{1});
end

function pose = readPoseFromUrdf(urdfPath)
urdfText = fileread(urdfPath);
jointBlock = extractTargetJointBlock(urdfText);
originTokens = regexp(jointBlock, '<origin\s+xyz="([^"]+)"\s+rpy="([^"]+)"\s*/>', 'tokens', 'once');
if isempty(originTokens)
    error('adjust_block_with_basin_pose_gui:OriginNotFound', ...
        'Origin not found in joint base_jizuo_to_block_with_basin_frame');
end

xyz = sscanf(originTokens{1}, '%f %f %f');
rpyRad = sscanf(originTokens{2}, '%f %f %f');

pose = struct( ...
    'x', xyz(1), ...
    'y', xyz(2), ...
    'z', xyz(3), ...
    'rollDeg', rad2deg(rpyRad(1)), ...
    'pitchDeg', rad2deg(rpyRad(2)), ...
    'yawDeg', rad2deg(rpyRad(3)));
end

function jointPosition = readInitialJointPosition(urdfPath)
urdfText = fileread(urdfPath);
ros2Blocks = regexp(urdfText, '<ros2_control[^>]*>[\s\S]*?</ros2_control>', 'match');
if isempty(ros2Blocks)
    error('adjust_block_with_basin_pose_gui:Ros2ControlBlockNotFound', ...
        'ros2_control block not found in URDF');
end
ros2Block = ros2Blocks{1};
jointNames = { ...
    'ur10_shoulder_pan', ...
    'ur10_shoulder_lift', ...
    'ur10_elbow', ...
    'ur10_wrist_1', ...
    'ur10_wrist_2', ...
    'ur10_wrist_3'};

jointPosition = zeros(1, numel(jointNames));
for i = 1:numel(jointNames)
    jointName = jointNames{i};
    pattern = ['<joint\s+name="' regexptranslate('escape', jointName) ...
        '"[^>]*>[\s\S]*?<state_interface\s+name="position">\s*' ...
        '<param\s+name="initial_value">([^<]+)</param>[\s\S]*?</joint>'];
    tokens = regexp(ros2Block, pattern, 'tokens', 'once');
    if isempty(tokens)
        error('adjust_block_with_basin_pose_gui:InitialJointValueNotFound', ...
            'Initial joint value not found for %s', jointName);
    end
    jointPosition(i) = str2double(strtrim(tokens{1}));
end
end

function jointBlock = extractTargetJointBlock(urdfText)
jointPattern = ['<joint\s+name="base_jizuo_to_block_with_basin_frame"\s+type="fixed">[\s\S]*?</joint>'];
jointBlock = regexp(urdfText, jointPattern, 'match', 'once');
if isempty(jointBlock)
    error('adjust_block_with_basin_pose_gui:JointNotFound', ...
        'Joint base_jizuo_to_block_with_basin_frame not found');
end
end

function previewText = buildPreviewUrdfText(urdfPath, pose)
urdfText = fileread(urdfPath);
previewText = replaceTargetJointOrigin(urdfText, pose);
end

function writePoseToUrdf(urdfPath, pose)
urdfText = fileread(urdfPath);
updatedText = replaceTargetJointOrigin(urdfText, pose);
writeTextFile(urdfPath, updatedText);
end

function updatedText = replaceTargetJointOrigin(urdfText, pose)
xyzText = sprintf('%.6f %.6f %.6f', pose.x, pose.y, pose.z);
rpyText = sprintf('%.12f %.12f %.12f', deg2rad(pose.rollDeg), deg2rad(pose.pitchDeg), deg2rad(pose.yawDeg));
jointPattern = '(<joint\s+name="base_jizuo_to_block_with_basin_frame"\s+type="fixed">[\s\S]*?<origin\s+xyz=")([^"]+)("\s+rpy=")([^"]+)("\s*/>[\s\S]*?</joint>)';
if isempty(regexp(urdfText, jointPattern, 'once'))
    error('adjust_block_with_basin_pose_gui:JointReplaceFailed', ...
        'Target joint origin pattern not found');
end
replaceText = ['$1' xyzText '$3' rpyText '$5'];
updatedText = regexprep(urdfText, jointPattern, replaceText, 'once');
end

function writePoseYaml(yamlPath, pose, sceneUrdfPath)
timestamp = char(datetime('now', 'Format', 'yyyy-MM-dd HH:mm:ss'));
lines = {
    'metadata:'
    sprintf('  exported_at: "%s"', timestamp)
    '  source_gui: "adjust_block_with_basin_pose_gui"'
    sprintf('  target_urdf: "%s"', sceneUrdfPath)
    'target_joint: "base_jizuo_to_block_with_basin_frame"'
    'pose:'
    sprintf('  x: %.6f', pose.x)
    sprintf('  y: %.6f', pose.y)
    sprintf('  z: %.6f', pose.z)
    sprintf('  roll_deg: %.6f', pose.rollDeg)
    sprintf('  pitch_deg: %.6f', pose.pitchDeg)
    sprintf('  yaw_deg: %.6f', pose.yawDeg)
    'meanings:'
    '  x: "block_with_basin 相对 base_jizuo 的 X 平移，单位 m"'
    '  y: "block_with_basin 相对 base_jizuo 的 Y 平移，单位 m"'
    '  z: "block_with_basin 相对 base_jizuo 的 Z 平移，单位 m"'
    '  roll_deg: "绕 X 轴旋转，单位 deg"'
    '  pitch_deg: "绕 Y 轴旋转，单位 deg"'
    '  yaw_deg: "绕 Z 轴旋转，单位 deg"'
    };
writeTextFile(yamlPath, sprintf('%s\n', lines{:}));
end

function writeTextFile(filePath, text)
fid = fopen(filePath, 'w', 'n', 'UTF-8');
if fid < 0
    error('adjust_block_with_basin_pose_gui:FileWriteFailed', ...
        'Unable to write file: %s', filePath);
end
cleanupObj = onCleanup(@() fclose(fid));
fwrite(fid, text, 'char');
clear cleanupObj;
end

function summary = buildPoseSummary(pose)
summary = sprintf(['xyz = [%.3f, %.3f, %.3f] m    ' ...
    'rpy = [%.1f, %.1f, %.1f] deg'], ...
    pose.x, pose.y, pose.z, pose.rollDeg, pose.pitchDeg, pose.yawDeg);
end
