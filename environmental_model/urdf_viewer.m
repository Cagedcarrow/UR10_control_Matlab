function urdf_viewer(urdf_path)
% URDF_VIEWER GUI for loading and inspecting URDF robot/environment models.
%
% Usage:
%   urdf_viewer                  Open empty viewer, browse for a URDF file
%   urdf_viewer('path/to/model.urdf')  Open and load the specified URDF

if nargin == 0
    urdf_path = '';
end

% ---- Build UI ----
fig = uifigure('Name', 'URDF Viewer', 'Position', [100 100 1200 750], ...
    'Color', [0.94 0.94 0.94]);

% Main layout: toolbar row | axes row | status row
main_grid = uigridlayout(fig, [3 1], ...
    'RowHeight', {40, '1x', 28}, ...
    'ColumnWidth', {'1x'}, ...
    'Padding', [8 8 8 5], 'RowSpacing', 5);

% -- Row 1: Toolbar --
toolbar = uigridlayout(main_grid, [1 9], ...
    'RowHeight', 34, ...
    'ColumnWidth', {100, 20, 55, 55, 55, 55, 50, 100, '1x'}, ...
    'Padding', [4 0 4 0]);

btn_open = uibutton(toolbar, 'push', 'Text', 'Open URDF...', ...
    'ButtonPushedFcn', @(src,evt) onOpen()); %#ok<NASGU>
uilabel(toolbar, 'Text', ''); % spacer

uilabel(toolbar, 'Text', 'Visuals:', 'HorizontalAlignment', 'right');
cb_visuals = uicheckbox(toolbar, 'Text', '', 'Value', 1, ...
    'ValueChangedFcn', @(src,evt) onToggle());

uilabel(toolbar, 'Text', 'Collisions:', 'HorizontalAlignment', 'right');
cb_collisions = uicheckbox(toolbar, 'Text', '', 'Value', 0, ...
    'ValueChangedFcn', @(src,evt) onToggle());

uilabel(toolbar, 'Text', 'Frames:', 'HorizontalAlignment', 'right');
cb_frames = uicheckbox(toolbar, 'Text', '', 'Value', 1, ...
    'ValueChangedFcn', @(src,evt) onToggle());

uilabel(toolbar, 'Text', 'World:', 'HorizontalAlignment', 'right');
cb_world = uicheckbox(toolbar, 'Text', '', 'Value', 1, ...
    'ValueChangedFcn', @(src,evt) onToggle());

dd_view = uidropdown(toolbar, ...
    'Items', {'Isometric', 'Top', 'Front', 'Side', 'Bottom'}, ...
    'Value', 'Isometric', ...
    'ValueChangedFcn', @(src,evt) onViewChange());

% -- Row 2: Axes panel --
pnl_axes = uipanel(main_grid, 'BorderType', 'none', ...
    'BackgroundColor', [0.97 0.97 0.97]);

% -- Row 3: Status bar --
status_bar = uigridlayout(main_grid, [1 2], ...
    'RowHeight', 22, ...
    'ColumnWidth', {'1x', 350}, ...
    'Padding', [4 0 4 0]);
lbl_status = uilabel(status_bar, 'Text', 'Ready. Open a URDF file to begin.', ...
    'HorizontalAlignment', 'left');
lbl_file = uilabel(status_bar, 'Text', '', 'HorizontalAlignment', 'right');

% ---- State ----
robot = [];

% ---- Initial load ----
if ~isempty(urdf_path)
    loadURDF(urdf_path);
end

% ===================== Nested Callbacks =====================

    function onOpen()
        [file, folder] = uigetfile({'*.urdf;*.xacro', 'URDF files (*.urdf,*.xacro)'; ...
            '*.*', 'All files (*.*)'}, 'Select URDF file');
        if isequal(file, 0)
            return;
        end
        loadURDF(fullfile(folder, file));
    end

    function loadURDF(filepath)
        lbl_status.Text = sprintf('Loading: %s ...', filepath);
        drawnow;

        [model_dir, ~, ~] = fileparts(filepath);
        mesh_candidate = fullfile(model_dir, 'meshes');
        if ~isfolder(mesh_candidate)
            mesh_candidate = model_dir;
        end

        try
            robot = importrobot(filepath, 'DataFormat', 'row', 'MeshPath', mesh_candidate);
            lbl_status.Text = sprintf('Loaded: %s  [%d bodies]', ...
                filepath, robot.NumBodies);
            lbl_file.Text = filepath;
            refreshAxes();
        catch ME
            lbl_status.Text = sprintf('Error: %s', ME.message);
            uialert(fig, ME.message, 'Load Error');
        end
    end

    function refreshAxes()
        delete(findobj(pnl_axes, 'Type', 'axes'));
        if isempty(robot)
            return;
        end
        ax = axes(pnl_axes); %#ok<LAXES>

        if cb_visuals.Value
            vis = 'on';
        else
            vis = 'off';
        end
        if cb_collisions.Value
            col = 'on';
        else
            col = 'off';
        end
        if cb_frames.Value
            frm = 'on';
        else
            frm = 'off';
        end

        show(robot, 'Visuals', vis, 'Collisions', col, 'Frames', frm, 'Parent', ax);
        axis(ax, 'equal');
        grid(ax, 'on');
        ax.GridColor = [0.85 0.85 0.85];

        % Draw world coordinate axes
        if cb_world.Value
            hold(ax, 'on');
            len = 0.3;
            quiver3(ax, 0, 0, 0, len, 0, 0, 'r', 'LineWidth', 2, 'MaxHeadSize', 0.6);
            quiver3(ax, 0, 0, 0, 0, len, 0, 'g', 'LineWidth', 2, 'MaxHeadSize', 0.6);
            quiver3(ax, 0, 0, 0, 0, 0, len, 'b', 'LineWidth', 2, 'MaxHeadSize', 0.6);
            text(len*1.1, 0, 0, 'X', 'Color', 'r', 'FontWeight', 'bold', 'FontSize', 10, 'Parent', ax);
            text(0, len*1.1, 0, 'Y', 'Color', 'g', 'FontWeight', 'bold', 'FontSize', 10, 'Parent', ax);
            text(0, 0, len*1.1, 'Z', 'Color', 'b', 'FontWeight', 'bold', 'FontSize', 10, 'Parent', ax);
            hold(ax, 'off');
        end

        applyView();
    end

    function onToggle()
        refreshAxes();
    end

    function onViewChange()
        applyView();
    end

    function applyView()
        ax = findobj(pnl_axes, 'Type', 'axes');
        if isempty(ax), return; end
        switch dd_view.Value
            case 'Top',       view(ax, 0, 90);
            case 'Front',     view(ax, 0, 0);
            case 'Side',      view(ax, 90, 0);
            case 'Bottom',    view(ax, 0, -90);
            case 'Isometric', view(ax, 135, 20);
        end
    end
end
