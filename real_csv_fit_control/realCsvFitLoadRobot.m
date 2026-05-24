function [robot, meta] = realCsvFitLoadRobot(cfg)
if ~isfile(cfg.xacroPath)
    error('未找到xacro文件: %s', cfg.xacroPath);
end

assemblyDir = fileparts(cfg.xacroPath);
meshDir = fullfile(assemblyDir, cfg.meshRootName);
raw = fileread(cfg.xacroPath);
raw = regexprep(raw, '<xacro:arg[^>]*/>\s*', '');
raw = strrep(raw, '$(arg mesh_root)', cfg.meshRootName);

tmpUrdf = fullfile(tempdir, 'assembly_preprocessed_real_csv_fit.urdf');
fid = fopen(tmpUrdf, 'w');
if fid < 0
    error('无法写入临时URDF: %s', tmpUrdf);
end
fwrite(fid, raw, 'char');
fclose(fid);

robot = importrobot(tmpUrdf, 'DataFormat', 'row', 'MeshPath', meshDir);

meta = struct();
meta.sourceXacro = cfg.xacroPath;
meta.meshDir = meshDir;
meta.endEffector = cfg.endEffector;
meta.bodyNames = robot.BodyNames;
end
