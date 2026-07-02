function validate_mud_smearing_model(robot, cfg)
%VALIDATE_MUD_SMEARING_MODEL Check required links, joints, and mesh files.

requiredLinks = [
    cfg.links.center
    cfg.links.forward
    cfg.links.left
    cfg.links.right
    cfg.links.wall
    cfg.links.block
    ];

bodyNames = string(robot.BodyNames);
missingLinks = requiredLinks(~ismember(requiredLinks, bodyNames));
if ~isempty(missingLinks)
    error("Required links are missing: %s", strjoin(missingLinks, ", "));
end

jointNames = list_movable_joint_names(robot);
if ~any(jointNames == "base_tail_to_shovel_base")
    error("Movable joint base_tail_to_shovel_base was not found.");
end

if numel(jointNames) ~= 7
    error("Expected 7 movable joints, found %d.", numel(jointNames));
end

urdfText = fileread(cfg.paths.urdfPath);
meshTokens = regexp(urdfText, '<mesh\s+filename="([^"]+)"', "tokens");
meshFiles = strings(1, numel(meshTokens));
for idx = 1:numel(meshTokens)
    meshFiles(idx) = string(meshTokens{idx}{1});
end
meshFiles = unique(meshFiles);
for idx = 1:numel(meshFiles)
    meshName = meshFiles(idx);
    if startsWith(meshName, "meshes/")
        meshPath = fullfile(cfg.paths.rootDir, meshName);
    else
        meshPath = fullfile(cfg.paths.meshDir, meshName);
    end
    if ~isfile(meshPath)
        error("Mesh file referenced by URDF is missing: %s", meshPath);
    end
end
end
