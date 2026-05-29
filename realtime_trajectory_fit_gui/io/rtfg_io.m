function varargout = rtfg_io(action, varargin)
switch action
    case 'buildPaths'
        varargout{1} = buildPaths(varargin{1});
    case 'ensureRequiredPaths'
        ensureRequiredPaths(varargin{1});
    case 'loadFallbackPose'
        varargout{1} = loadFallbackPose(varargin{1});
    case 'readRuntimeConfigYaml'
        [varargout{1}, varargout{2}] = readRuntimeConfigYaml(varargin{1}, varargin{2}, varargin{3});
    case 'writeRuntimeConfigYaml'
        writeRuntimeConfigYaml(varargin{1}, varargin{2});
    case 'readPoseFromYaml'
        varargout{1} = readPoseFromYaml(varargin{1});
    case 'readPoseFromUrdf'
        varargout{1} = readPoseFromUrdf(varargin{1});
    case 'readInitialJointPosition'
        varargout{1} = readInitialJointPosition(varargin{1});
    case 'buildPreviewUrdfText'
        varargout{1} = buildPreviewUrdfText(varargin{1}, varargin{2});
    case 'writePoseToUrdf'
        writePoseToUrdf(varargin{1}, varargin{2});
    case 'writeTextFile'
        writeTextFile(varargin{1}, varargin{2});
    otherwise
        error('rtfg_io:UnknownAction', 'Unknown action: %s', action);
end
end

function paths = buildPaths(thisDir)
repoRoot = fileparts(thisDir);
paths = struct();
paths.thisDir = thisDir;
paths.repoRoot = repoRoot;
paths.sceneUrdf = fullfile(repoRoot, 'environmental_model', 'assembly_with_block_with_basin.urdf');
paths.collisionRobotUrdf = fullfile(repoRoot, 'environmental_model', 'ur10_shovel_only.urdf');
paths.meshDir = fullfile(repoRoot, 'environmental_model', 'meshes');
paths.environmentPoseYaml = fullfile(repoRoot, 'environmental_model', 'block_with_basin_pose.yaml');
paths.trajectoryGuiDir = fullfile(repoRoot, 'test', 'tarjectory_plan', 'trajectory_plan_3d_gui');
paths.trajectoryDefaultsYaml = fullfile(paths.trajectoryGuiDir, 'trajectory_params_3d.yaml');
paths.runtimeYaml = fullfile(thisDir, 'environment_runtime_config.yaml');
end

function ensureRequiredPaths(paths)
requiredPaths = {paths.sceneUrdf, paths.collisionRobotUrdf, paths.meshDir, paths.trajectoryGuiDir, paths.trajectoryDefaultsYaml};
for i = 1:numel(requiredPaths)
    thisPath = requiredPaths{i};
    if ~exist(thisPath, 'file') && ~exist(thisPath, 'dir')
        error('main_realtime_trajectory_fit_gui:MissingPath', ...
            'Required path not found: %s', thisPath);
    end
end
end

function pose = loadFallbackPose(paths)
if isfile(paths.environmentPoseYaml)
    pose = readPoseFromYaml(paths.environmentPoseYaml);
else
    pose = readPoseFromUrdf(paths.sceneUrdf);
end
end

function [trajParams, pose] = readRuntimeConfigYaml(yamlPath, defaultTrajParams, defaultPose)
trajParams = defaultTrajParams;
pose = defaultPose;

lines = splitlines(string(fileread(yamlPath)));
section = "";
subsection = "";

for i = 1:numel(lines)
    rawLine = lines(i);
    trimmed = strtrim(rawLine);
    if trimmed == "" || startsWith(trimmed, "#")
        continue;
    end

    indent = strlength(rawLine) - strlength(strip(rawLine, 'left'));

    if indent == 0
        subsection = "";
        if trimmed == "trajectory:"
            section = "trajectory";
            continue;
        elseif trimmed == "environment_pose:"
            section = "environment_pose";
            continue;
        else
            section = "";
            continue;
        end
    end

    if indent == 2
        if section == "trajectory" && trimmed == "parameters:"
            subsection = "trajectory_parameters";
            continue;
        elseif section == "environment_pose" && trimmed == "pose:"
            subsection = "environment_pose";
            continue;
        else
            subsection = "";
            continue;
        end
    end

    tokens = regexp(trimmed, '^([A-Za-z0-9_]+):\s*([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)$', 'tokens', 'once');
    if isempty(tokens)
        continue;
    end

    key = char(tokens{1});
    value = str2double(tokens{2});

    switch subsection
        case 'trajectory_parameters'
            switch key
                case 'left_wall_offset'
                    trajParams.leftWallOffset = value;
                case 'mud_height'
                    trajParams.mudHeight = value;
                case 'approach_len'
                    trajParams.approachLen = value;
                case 'theta_deg'
                    trajParams.thetaDeg = value;
                case 'depth'
                    trajParams.depth = value;
                case 'x_plane'
                    trajParams.xPlane = value;
            end
        case 'environment_pose'
            switch key
                case 'x'
                    pose.x = value;
                case 'y'
                    pose.y = value;
                case 'z'
                    pose.z = value;
                case 'roll_deg'
                    pose.rollDeg = value;
                case 'pitch_deg'
                    pose.pitchDeg = value;
                case 'yaw_deg'
                    pose.yawDeg = value;
            end
    end
end
end

function writeRuntimeConfigYaml(yamlPath, state)
traj = state.traj;
timestamp = char(datetime('now', 'Format', 'yyyy-MM-dd HH:mm:ss'));
lines = {
    'metadata:'
    sprintf('  exported_at: "%s"', timestamp)
    '  source_gui: "main_realtime_trajectory_fit_gui"'
    sprintf('  target_urdf: "%s"', state.paths.sceneUrdf)
    'trajectory:'
    '  parameters:'
    sprintf('    left_wall_offset: %.6f', state.trajParams.leftWallOffset)
    sprintf('    mud_height: %.6f', state.trajParams.mudHeight)
    sprintf('    approach_len: %.6f', state.trajParams.approachLen)
    sprintf('    theta_deg: %.6f', state.trajParams.thetaDeg)
    sprintf('    depth: %.6f', state.trajParams.depth)
    sprintf('    x_plane: %.6f', state.trajParams.xPlane)
    '  derived_values:'
    sprintf('    approach_start_point_xyz: [%.6f, %.6f, %.6f]', traj.pApproachStart(1), traj.pApproachStart(2), traj.pApproachStart(3))
    sprintf('    entry_point_xyz: [%.6f, %.6f, %.6f]', traj.pEntry(1), traj.pEntry(2), traj.pEntry(3))
    sprintf('    arc_end_point_xyz: [%.6f, %.6f, %.6f]', traj.pArcEnd(1), traj.pArcEnd(2), traj.pArcEnd(3))
    sprintf('    approach_length: %.6f', traj.approachLen)
    sprintf('    arc_radius: %.6f', traj.arcRadius)
    sprintf('    vertical_penetration: %.6f', traj.verticalPenetration)
    sprintf('    dist_x_negative_wall: %.6f', traj.distLeft)
    sprintf('    dist_x_positive_wall: %.6f', traj.distRight)
    'environment_pose:'
    '  target_joint: "base_jizuo_to_block_with_basin_frame"'
    '  pose:'
    sprintf('    x: %.6f', state.pose.x)
    sprintf('    y: %.6f', state.pose.y)
    sprintf('    z: %.6f', state.pose.z)
    sprintf('    roll_deg: %.6f', state.pose.rollDeg)
    sprintf('    pitch_deg: %.6f', state.pose.pitchDeg)
    sprintf('    yaw_deg: %.6f', state.pose.yawDeg)
    'meanings:'
    '  left_wall_offset: "入泥点到左侧壁的水平距离，单位 m"'
    '  mud_height: "泥面距盆底的高度，单位 m"'
    '  approach_len: "起始运动点到入泥点的斜线长度，单位 m"'
    '  theta_deg: "入泥角，负值表示向下切入，单位 deg"'
    '  depth: "入泥点相对泥面的实际垂直下扎深度，单位 m"'
    '  x_plane: "当前 YOZ 轨迹所在的世界 X 截面位置，单位 m"'
    '  x: "block_with_basin 相对 base_jizuo 的 X 平移，单位 m"'
    '  y: "block_with_basin 相对 base_jizuo 的 Y 平移，单位 m"'
    '  z: "block_with_basin 相对 base_jizuo 的 Z 平移，单位 m"'
    '  roll_deg: "绕 X 轴旋转，单位 deg"'
    '  pitch_deg: "绕 Y 轴旋转，单位 deg"'
    '  yaw_deg: "绕 Z 轴旋转，单位 deg"'
    };
writeTextFile(yamlPath, sprintf('%s\n', lines{:}));
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
    error('main_realtime_trajectory_fit_gui:YamlParseFailed', 'Key not found in YAML: %s', key);
end
value = str2double(tokens{1});
end

function pose = readPoseFromUrdf(urdfPath)
urdfText = fileread(urdfPath);
jointBlock = extractTargetJointBlock(urdfText);
originTokens = regexp(jointBlock, '<origin\s+xyz="([^"]+)"\s+rpy="([^"]+)"\s*/>', 'tokens', 'once');
if isempty(originTokens)
    error('main_realtime_trajectory_fit_gui:OriginNotFound', ...
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

function q = readInitialJointPosition(urdfPath)
urdfText = fileread(urdfPath);
ros2Blocks = regexp(urdfText, '<ros2_control[^>]*>[\s\S]*?</ros2_control>', 'match');
if isempty(ros2Blocks)
    error('main_realtime_trajectory_fit_gui:Ros2ControlBlockNotFound', ...
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
q = zeros(1, numel(jointNames));
for i = 1:numel(jointNames)
    jointName = jointNames{i};
    pattern = ['<joint\s+name="' regexptranslate('escape', jointName) ...
        '"[^>]*>[\s\S]*?<state_interface\s+name="position">\s*' ...
        '<param\s+name="initial_value">([^<]+)</param>[\s\S]*?</joint>'];
    tokens = regexp(ros2Block, pattern, 'tokens', 'once');
    if isempty(tokens)
        error('main_realtime_trajectory_fit_gui:InitialJointValueNotFound', ...
            'Initial joint value not found for %s', jointName);
    end
    q(i) = str2double(strtrim(tokens{1}));
end
end

function jointBlock = extractTargetJointBlock(urdfText)
jointPattern = '<joint\s+name="base_jizuo_to_block_with_basin_frame"\s+type="fixed">[\s\S]*?</joint>';
jointBlock = regexp(urdfText, jointPattern, 'match', 'once');
if isempty(jointBlock)
    error('main_realtime_trajectory_fit_gui:JointNotFound', ...
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
    error('main_realtime_trajectory_fit_gui:JointReplaceFailed', ...
        'Target joint origin pattern not found');
end
updatedText = regexprep(urdfText, jointPattern, ['$1' xyzText '$3' rpyText '$5'], 'once');
end

function writeTextFile(filePath, text)
fid = fopen(filePath, 'w', 'n', 'UTF-8');
if fid < 0
    error('main_realtime_trajectory_fit_gui:FileWriteFailed', ...
        'Unable to write file: %s', filePath);
end
cleanupObj = onCleanup(@() fclose(fid));
fwrite(fid, text, 'char');
clear cleanupObj;
end
