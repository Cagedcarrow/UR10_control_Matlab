function [robot, cfg] = load_mud_smearing_robot(cfg)
%LOAD_MUD_SMEARING_ROBOT Import the local URDF as a rigidBodyTree.

if ~isfile(cfg.paths.urdfPath)
    error("URDF file not found: %s", cfg.paths.urdfPath);
end

if ~isfolder(cfg.paths.meshDir)
    error("Mesh directory not found: %s", cfg.paths.meshDir);
end

robot = importrobot(cfg.paths.urdfPath, ...
    "DataFormat", "row", ...
    "MeshPath", cfg.paths.meshDir);
end
