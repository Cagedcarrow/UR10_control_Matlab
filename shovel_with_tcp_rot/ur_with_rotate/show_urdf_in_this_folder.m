function robot = show_urdf_in_this_folder(urdfFile)
%SHOW_URDF_IN_THIS_FOLDER Pick a URDF file and display it.

scriptDir = fileparts(mfilename("fullpath"));
if nargin < 1 || strlength(string(urdfFile)) == 0
    [selectedFile, selectedDir] = uigetfile( ...
        {"*.urdf", "URDF files (*.urdf)"; "*.*", "All files (*.*)"}, ...
        "Select URDF file", ...
        scriptDir);
    if isequal(selectedFile, 0)
        robot = [];
        fprintf("URDF selection canceled.\n");
        return;
    end
    urdfPath = fullfile(selectedDir, selectedFile);
else
    urdfPath = char(urdfFile);
    if ~isfile(urdfPath)
        urdfPath = fullfile(scriptDir, urdfPath);
    end
end

[urdfDir, urdfBaseName, ~] = fileparts(urdfPath);
[packageDir, meshDir] = inferPackageAndMeshDirs(urdfDir);

if ~isfile(urdfPath)
    error("URDF file not found: %s", urdfPath);
end

if ~isfolder(meshDir)
    error("Mesh folder not found. Expected: %s", meshDir);
end

fprintf("Loading URDF: %s\n", urdfPath);
rawUrdf = fileread(urdfPath);
rawUrdf = resolvePackageUris(rawUrdf, packageDir);
tmpUrdfPath = fullfile(tempdir, sprintf("%s_show_urdf.urdf", urdfBaseName));
fid = fopen(tmpUrdfPath, "w");
if fid < 0
    error("Cannot write temporary URDF: %s", tmpUrdfPath);
end
cleanupFile = onCleanup(@() deleteTempFile(tmpUrdfPath));
cleanupFid = onCleanup(@() fclose(fid));
fwrite(fid, rawUrdf, "char");
clear cleanupFid;

robot = importrobot(tmpUrdfPath, "DataFormat", "row", "MeshPath", meshDir);

figure("Name", "URDF viewer", "Color", "w");
ax = axes;
show(robot, homeConfiguration(robot), ...
    "Parent", ax, ...
    "Visuals", "on", ...
    "Collisions", "off", ...
    "Frames", "on");
axis(ax, "equal");
grid(ax, "on");
view(ax, 135, 20);
title(ax, strrep(urdfBaseName, "_", "\_"));
end

function [packageDir, meshDir] = inferPackageAndMeshDirs(urdfDir)
sameLevelMeshDir = fullfile(urdfDir, "meshes");
parentDir = fileparts(urdfDir);
parentMeshDir = fullfile(parentDir, "meshes");

if isfolder(sameLevelMeshDir)
    packageDir = urdfDir;
    meshDir = sameLevelMeshDir;
elseif isfolder(parentMeshDir)
    packageDir = parentDir;
    meshDir = parentMeshDir;
else
    packageDir = parentDir;
    meshDir = sameLevelMeshDir;
end
end

function rawUrdf = resolvePackageUris(rawUrdf, packageDir)
rawUrdf = regexprep(rawUrdf, "package://[^/]+", escapeReplacement(packageDir));
end

function replacement = escapeReplacement(text)
replacement = strrep(text, "\", "\\");
replacement = strrep(replacement, "$", "$$");
end

function deleteTempFile(filePath)
if isfile(filePath)
    delete(filePath);
end
end
