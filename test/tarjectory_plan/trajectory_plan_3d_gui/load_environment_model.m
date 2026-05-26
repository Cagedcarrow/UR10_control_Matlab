function robot = load_environment_model(urdfPath)
%LOAD_ENVIRONMENT_MODEL Load the 3D block + basin environment.

if ~isfile(urdfPath)
    error('未找到环境 URDF 文件: %s', urdfPath);
end

modelDir = fileparts(urdfPath);
meshCandidate = fullfile(modelDir, 'meshes');
if ~isfolder(meshCandidate)
    meshCandidate = modelDir;
end

robot = importrobot(urdfPath, 'DataFormat', 'row', 'MeshPath', meshCandidate);
end
