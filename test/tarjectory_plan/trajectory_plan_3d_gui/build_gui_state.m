function state = build_gui_state()
%BUILD_GUI_STATE Create defaults and environment geometry for the 3D GUI.

thisDir = fileparts(mfilename('fullpath'));
trajectoryPlanDir = fileparts(thisDir);
projectRoot = fileparts(fileparts(fileparts(trajectoryPlanDir)));

state = struct();
state.paths = struct( ...
    'thisDir', thisDir, ...
    'trajectoryPlanDir', trajectoryPlanDir, ...
    'projectRoot', projectRoot, ...
    'environmentUrdf', fullfile(projectRoot, 'environmental_model', 'block_with_basin.urdf'));

basin = struct();
basin.worldOrigin = [-0.135, 0.0, 0.25];
basin.outerLengthX = 0.37;
basin.outerWidthY = 0.50;
basin.outerHeightZ = 0.18;
basin.wallThickness = 0.003;
basin.innerLengthX = basin.outerLengthX - 2 * basin.wallThickness;
basin.innerWidthY = basin.outerWidthY - 2 * basin.wallThickness;
basin.innerHeightZ = basin.outerHeightZ;
basin.innerXMin = basin.worldOrigin(1) - basin.innerLengthX / 2;
basin.innerXMax = basin.worldOrigin(1) + basin.innerLengthX / 2;
basin.innerYMin = basin.worldOrigin(2) - basin.innerWidthY / 2;
basin.innerYMax = basin.worldOrigin(2) + basin.innerWidthY / 2;
basin.floorZ = basin.worldOrigin(3);
basin.rimZ = basin.worldOrigin(3) + basin.innerHeightZ;
basin.mudSurfaceZ = basin.floorZ + 0.60 * basin.innerHeightZ;
basin.innerMargin = 0.01;
basin.xSafetyMargin = 0.01;
basin.ySafetyMargin = 0.01;
basin.xPlaneMin = basin.innerXMin + basin.xSafetyMargin;
basin.xPlaneMax = basin.innerXMax - basin.xSafetyMargin;
basin.yMotionMin = basin.innerYMin + basin.ySafetyMargin;
basin.yMotionMax = basin.innerYMax - basin.ySafetyMargin;
basin.zMotionMin = basin.floorZ + basin.innerMargin;
basin.zMotionMax = basin.rimZ - basin.innerMargin;

state.envGeom = struct();
state.envGeom.block = struct('size', [1.0, 1.0, 0.25], 'center', [0.0, 0.0, 0.125]);
state.envGeom.basin = basin;
state.envGeom.motion = struct('approachLen', 0.08);

state.params = struct( ...
    'leftWallOffset', 0.247, ...
    'mudHeight', 0.108, ...
    'thetaDeg', -30.0, ...
    'depth', 0.05, ...
    'xPlane', basin.worldOrigin(1));

state.robot = [];
state.ui = struct();
state.ui.cameraState = struct('mode', 'preset', 'value', [135, 20]);
state.traj = [];
end
