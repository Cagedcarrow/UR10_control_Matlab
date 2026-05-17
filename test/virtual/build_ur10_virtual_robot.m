function [robot, meta] = build_ur10_virtual_robot(opts)
% 从 assembly.urdf.xacro 导入并裁剪为固定虚拟链

if nargin < 1
    opts = init_virtual_ur10(fileparts(mfilename('fullpath')));
end
if ~isfile(opts.xacroPath)
    error('未找到xacro文件: %s', opts.xacroPath);
end

srcRobot = importFromXacro(opts.xacroPath, opts.meshDir);
robot = rigidBodyTree('DataFormat','row','MaxNumBodies',8);

bodyOrder = { ...
    'ur10_shoulder', ...
    'ur10_upper_arm', ...
    'ur10_forearm', ...
    'ur10_wrist_1', ...
    'ur10_wrist_2', ...
    'ur10_wrist_3', ...
    'sensor_shovel', ...
    'sensor_shovel_tcp'};

parentMap = containers.Map( ...
    bodyOrder, ...
    {'base','ur10_shoulder','ur10_upper_arm','ur10_forearm','ur10_wrist_1','ur10_wrist_2','ur10_wrist_3','sensor_shovel'});

for i = 1:numel(bodyOrder)
    bodyName = bodyOrder{i};
    srcBody = srcRobot.getBody(bodyName);
    newBody = srcBody.copy();

    if strcmp(bodyName, 'ur10_shoulder')
        Tmount = trvec2tform([0 0 opts.baseHeight]) * eul2tform(opts.mountRPY, 'XYZ');
        j = newBody.Joint;
        setFixedTransform(j, Tmount * j.JointToParentTransform);
        newBody.Joint = j;
    end

    if strcmp(newBody.Joint.Type, 'revolute')
        idx = find(strcmp(opts.jointNames, newBody.Joint.Name), 1);
        if ~isempty(idx)
            newBody.Joint.PositionLimits = opts.jointLimitsRad(idx,:);
        end
    end

    addBody(robot, newBody, parentMap(bodyName));
end

jointNames = {};
for i = 1:numel(robot.Bodies)
    jn = robot.Bodies{i}.Joint.Name;
    if ~isempty(jn) && ~strcmp(robot.Bodies{i}.Joint.Type, 'fixed')
        jointNames{end+1} = jn; %#ok<AGROW>
    end
end

meta = struct();
meta.endEffector = 'sensor_shovel_tcp';
meta.bodyNames = [{'ur10'}; robot.BodyNames(:)];
meta.jointNames = jointNames;
meta.sourceXacro = opts.xacroPath;

expectedJoints = {'ur10_shoulder_pan','ur10_shoulder_lift','ur10_elbow','ur10_wrist_1','ur10_wrist_2','ur10_wrist_3'};
if numel(jointNames) ~= 6 || ~isequal(jointNames, expectedJoints)
    error('关节链校验失败，当前关节顺序不符合要求。');
end
end

function robot = importFromXacro(xacroPath, meshDir)
raw = fileread(xacroPath);
raw = regexprep(raw, '<xacro:arg[^>]*/>\s*', '');
raw = strrep(raw, '$(arg mesh_root)', 'meshes');

tmpUrdf = fullfile(tempdir, 'ur10_virtual_from_xacro.urdf');
fid = fopen(tmpUrdf, 'w');
if fid < 0
    error('无法写入临时URDF: %s', tmpUrdf);
end
c = onCleanup(@() fclose(fid)); %#ok<NASGU>
fwrite(fid, raw, 'char');

robot = importrobot(tmpUrdf, 'DataFormat', 'row', 'MeshPath', meshDir);
end
