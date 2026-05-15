function show_assembly_urdf()
% SHOW_ASSEMBLY_URDF Preprocess xacro-like URDF and display it in MATLAB.

clc;

xacroPath = 'E:/UR10_control/assembly/assembly.urdf.xacro';
meshRootName = 'meshes';

fprintf('[INFO] xacro path: %s\n', xacroPath);
if ~isfile(xacroPath)
    error('[ERROR] File not found: %s', xacroPath);
end

assemblyDir = fileparts(xacroPath);
meshDir = fullfile(assemblyDir, meshRootName);
fprintf('[INFO] mesh dir: %s\n', meshDir);
if ~isfolder(meshDir)
    warning('[WARN] Mesh directory does not exist: %s', meshDir);
end

raw = fileread(xacroPath);

% Remove xacro arg declaration and resolve mesh_root usage.
raw = regexprep(raw, '<xacro:arg[^>]*/>\s*', '');
raw = strrep(raw, '$(arg mesh_root)', meshRootName);

tmpUrdf = fullfile(tempdir, 'assembly_preprocessed.urdf');
fid = fopen(tmpUrdf, 'w');
if fid < 0
    error('[ERROR] Cannot write temp URDF: %s', tmpUrdf);
end
cleanupObj = onCleanup(@() fclose(fid)); %#ok<NASGU>
fwrite(fid, raw, 'char');

fprintf('[INFO] temp urdf: %s\n', tmpUrdf);

try
    robot = importrobot(tmpUrdf, 'DataFormat', 'row', 'MeshPath', meshDir);
catch ME
    fprintf(2, '[ERROR] importrobot failed: %s\n', ME.message);
    fprintf('[DEBUG] temp urdf kept at: %s\n', tmpUrdf);
    rethrow(ME);
end

figure('Name', 'assembly URDF Preview', 'Color', 'w');
ax = axes; %#ok<LAXES>
show(robot, 'Visuals', 'on', 'Collisions', 'off', 'Frames', 'off', 'Parent', ax);
axis(ax, 'equal');
grid(ax, 'on');
view(ax, 135, 20);
title(ax, 'assembly.urdf.xacro (preprocessed)');

fprintf('[INFO] Done. If mesh is missing, check file names under: %s\n', meshDir);
end
