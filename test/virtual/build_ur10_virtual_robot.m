function [robot, meta] = build_ur10_virtual_robot(opts)
% 从 assembly.urdf.xacro 导入真实机械臂同源模型（与 ur10_gui_control.m 一致）

if nargin < 1
    opts = init_virtual_ur10(fileparts(mfilename('fullpath')));
end
if ~isfile(opts.xacroPath)
    error('未找到xacro文件: %s', opts.xacroPath);
end

robot = importFromXacro(opts.xacroPath, opts.meshDir);

jointNames = strings(1, 0);
jointBodyNames = strings(1, 0);
for i = 1:numel(robot.Bodies)
    jt = robot.Bodies{i}.Joint;
    if strcmp(jt.Type, 'revolute')
        jointNames(end+1) = string(jt.Name); %#ok<AGROW>
        jointBodyNames(end+1) = string(robot.Bodies{i}.Name); %#ok<AGROW>
    end
end

if numel(jointNames) ~= 6
    error('导入模型的可动关节数为 %d，期望为 6。', numel(jointNames));
end

jointCfgIdx = zeros(1, 6);
cfgNames = jointNames;
for i = 1:6
    idx = find(cfgNames == string(opts.jointNames{i}), 1);
    if isempty(idx)
        error('在导入模型中未找到关节: %s', opts.jointNames{i});
    end
    jointCfgIdx(i) = idx;
end

meta = struct();
meta.endEffector = 'sensor_shovel_tcp';
meta.bodyNames = [{'ur10'}; robot.BodyNames(:)];
meta.jointNames = cellstr(jointNames);
meta.jointBodyNames = cellstr(jointBodyNames);
meta.jointCfgIdx = jointCfgIdx;
meta.sourceXacro = opts.xacroPath;
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
