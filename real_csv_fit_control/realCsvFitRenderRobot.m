function realCsvFitRenderRobot(ax, robot, qrow, showPath, qPath, cfg)
% 与 test/virtual/nihe_gui.m 保持一致：单姿态 show，不叠加半透明模型。
if nargin < 4
    showPath = false;
end
if nargin < 5
    qPath = [];
end
if ~isgraphics(ax)
    return;
end

try
    show(robot, qrow, 'Parent', ax, ...
        'Visuals', 'on', 'Collisions', 'off', 'Frames', 'off', ...
        'PreservePlot', false, 'FastUpdate', true);
    axis(ax, 'equal');
    grid(ax, 'on');
    view(ax, cfg.guiView(1), cfg.guiView(2));
    realCsvFitDisableAxesInteractions(ax);

    if showPath && ~isempty(qPath)
        hold(ax, 'on');
        pts = localTcpPath(robot, qPath, cfg);
        if ~isempty(pts)
            plot3(ax, pts(:,1), pts(:,2), pts(:,3), 'b-', 'LineWidth', 1.2);
        end
        hold(ax, 'off');
    end

    drawnow limitrate nocallbacks;
catch ME
    fprintf(2, '[REAL_CSV_FIT][WARN] render failed: %s\n', ME.message);
end
end

function pts = localTcpPath(robot, qPath, cfg)
n = size(qPath,1);
if n < 1
    pts = zeros(0,3);
    return;
end

idx = unique(round(linspace(1, n, min(n, cfg.pathMaxPoints))));
pts = zeros(numel(idx), 3);
for k = 1:numel(idx)
    try
        tf = getTransform(robot, qPath(idx(k),:), cfg.endEffector);
        pts(k,:) = tform2trvec(tf);
    catch
        pts = zeros(0,3);
        return;
    end
end
end
