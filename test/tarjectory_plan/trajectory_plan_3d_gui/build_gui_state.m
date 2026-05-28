function state = build_gui_state(initialYamlName)
%BUILD_GUI_STATE Create defaults and environment geometry for the 3D GUI.

if nargin < 1
    initialYamlName = '';
end

thisDir = fileparts(mfilename('fullpath'));
trajectoryPlanDir = fileparts(thisDir);
projectRoot = fileparts(fileparts(trajectoryPlanDir));

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

state.params = struct( ...
    'leftWallOffset', 0.247, ...
    'mudHeight', 0.108, ...
    'approachLen', 0.08, ...
    'thetaDeg', -30.0, ...
    'depth', 0.05, ...
    'xPlane', basin.worldOrigin(1));

if ~isempty(initialYamlName)
    state.paths.initialParamsYaml = fullfile(thisDir, initialYamlName);
    if exist(state.paths.initialParamsYaml, 'file')
        state.params = applyInitialParamsFromYaml(state.params, state.paths.initialParamsYaml);
    end
else
    state.paths.initialParamsYaml = '';
end

state.params = clampParamsToGuiLimits(state.params, basin);

state.robot = [];
state.ui = struct();
state.ui.cameraState = struct('mode', 'preset', 'value', [135, 20]);
state.traj = [];
end

function params = applyInitialParamsFromYaml(params, yamlPath)
yamlText = fileread(yamlPath);
yamlLines = splitlines(string(yamlText));

keyMap = struct( ...
    'left_wall_offset', 'leftWallOffset', ...
    'mud_height', 'mudHeight', ...
    'approach_len', 'approachLen', ...
    'theta_deg', 'thetaDeg', ...
    'depth', 'depth', ...
    'x_plane', 'xPlane');

inParameters = false;
for i = 1:numel(yamlLines)
    line = yamlLines(i);
    trimmed = strtrim(line);
    if trimmed == ""
        continue;
    end

    if strcmp(trimmed, "parameters:")
        inParameters = true;
        continue;
    end

    if inParameters && ~startsWith(line, "  ")
        break;
    end

    if ~inParameters
        continue;
    end

    tokens = regexp(trimmed, '^([a-z_]+):\s*([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)$', 'tokens', 'once');
    if isempty(tokens)
        continue;
    end

    yamlKey = char(tokens{1});
    if isfield(keyMap, yamlKey)
        params.(keyMap.(yamlKey)) = str2double(tokens{2});
    end
end
end

function params = clampParamsToGuiLimits(params, basin)
params.leftWallOffset = clampValue(params.leftWallOffset, [basin.ySafetyMargin, basin.innerWidthY - basin.ySafetyMargin]);
params.mudHeight = clampValue(params.mudHeight, [0.00, basin.innerHeightZ]);
params.approachLen = clampValue(params.approachLen, [0.00, 0.30]);
params.thetaDeg = clampValue(params.thetaDeg, [-60.0, -5.0]);
params.depth = clampValue(params.depth, [0.01, 0.10]);
params.xPlane = clampValue(params.xPlane, [basin.xPlaneMin, basin.xPlaneMax]);
end

function value = clampValue(value, limits)
value = min(max(value, limits(1)), limits(2));
end
