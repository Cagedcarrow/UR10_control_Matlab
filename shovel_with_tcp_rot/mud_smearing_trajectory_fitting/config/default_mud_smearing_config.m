function cfg = default_mud_smearing_config()
%DEFAULT_MUD_SMEARING_CONFIG Parameters for mud smearing trajectory fitting.

rootDir = fileparts(fileparts(mfilename("fullpath")));

cfg = struct();
cfg.paths.rootDir = rootDir;
cfg.paths.urdfPath = fullfile(rootDir, "claying_shovel_env.urdf");
cfg.paths.meshDir = fullfile(rootDir, "meshes");

cfg.links.center = "shovel_surface_center_link";
cfg.links.forward = "shovel_forward_link";
cfg.links.left = "shovel_left_link";
cfg.links.right = "shovel_right_link";
cfg.links.wall = "block_basin_back_wall";
cfg.links.block = "block_basin_block";

cfg.wall.size = [0.10, 0.003, 0.60];
cfg.block.size = [1.0, 1.0, 0.25];

cfg.trajectory.startClearanceAboveBlock = 0.20;
cfg.trajectory.endDetachMargin = 0.00;
cfg.trajectory.numWaypoints = 25;
cfg.trajectory.approachSteps = 35;
cfg.trajectory.desiredCenterXAxis = [0; 0; 1];
cfg.trajectory.desiredCenterYAxis = [-1; 0; 0];
cfg.trajectory.desiredCenterZAxis = [0; -1; 0];

cfg.ik.weightsList = [
    1, 1, 1, 0.25, 0.25, 0.25
    1, 1, 1, 0.10, 0.10, 0.10
    1, 1, 1, 0.03, 0.03, 0.03
    ];
cfg.ik.positionTolerance = 0.02;
cfg.ik.centerAxisToleranceDeg = 25;

cfg.initialJointValues = struct( ...
    "ur10_shoulder_pan", 2.0474984645843506, ...
    "ur10_shoulder_lift", 0.21928656101226807, ...
    "ur10_elbow", -1.9548214117633265, ...
    "ur10_wrist_1", -0.35923797289003545, ...
    "ur10_wrist_2", 2.0502676963806152, ...
    "ur10_wrist_3", 1.0330829620361328, ...
    "base_tail_to_shovel_base", 0);

cfg.visual.showAnimation = true;
cfg.visual.shovelPlaneAlpha = 0.35;
cfg.visual.wallAlpha = 0.25;

cfg.ui.showUi = true;
end
