function plot_trajectory(ax, traj)
% 绘制泥面、三段轨迹和关键参数标注

cla(ax);
hold(ax, 'on');

xAll = traj.all(:,1);
yAll = traj.all(:,2);

xMin = min(xAll) - 0.35;
xMax = max(xAll) + 0.35;
plot(ax, [xMin xMax], [0 0], '--', 'Color', [0.10 0.40 0.95], ...
    'LineWidth', 1.6, 'DisplayName', '泥面 Mud Surface');

plot(ax, traj.seg1(:,1), traj.seg1(:,2), '-', 'Color', [0.85 0.25 0.25], ...
    'LineWidth', 2.4, 'DisplayName', '阶段1: 直线切入');
plot(ax, traj.seg2(:,1), traj.seg2(:,2), '-', 'Color', [0.15 0.45 0.90], ...
    'LineWidth', 2.4, 'DisplayName', '阶段2: 圆弧装载');
plot(ax, traj.seg3(:,1), traj.seg3(:,2), '-', 'Color', [0.12 0.62 0.32], ...
    'LineWidth', 2.4, 'DisplayName', '阶段3: 竖直抬升');

idx = round(linspace(1, size(traj.all,1), 14));
idx = unique(idx(:));
arrowLen = 0.08;
qx = arrowLen * cos(traj.heading(idx));
qy = arrowLen * sin(traj.heading(idx));
quiver(ax, traj.all(idx,1), traj.all(idx,2), qx, qy, 0, ...
    'Color', [0.18 0.18 0.18], 'LineWidth', 1.0, 'MaxHeadSize', 1.8, ...
    'DisplayName', '铲子姿态方向');

plot(ax, traj.p0(1), traj.p0(2), 'ko', 'MarkerFaceColor', 'k', 'HandleVisibility', 'off');
plot(ax, traj.p1(1), traj.p1(2), 'ko', 'MarkerFaceColor', [0.85 0.25 0.25], 'HandleVisibility', 'off');
plot(ax, traj.p2(1), traj.p2(2), 'ko', 'MarkerFaceColor', [0.15 0.45 0.90], 'HandleVisibility', 'off');

dimXh = traj.p0(1) / 2;
plot(ax, [0 traj.p0(1)], [0 0], '-', 'Color', [0.35 0.35 0.35], ...
    'LineWidth', 1.1, 'HandleVisibility', 'off');
text(ax, dimXh, 0.02, sprintf('x0 = %.3f m', traj.x0), ...
    'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom', 'FontSize', 10, ...
    'Color', [0.20 0.20 0.20]);

dimXz = traj.p0(1) - 0.10;
plot(ax, [dimXz dimXz], [0 traj.p0(2)], '-', 'Color', [0.35 0.35 0.35], ...
    'LineWidth', 1.1, 'HandleVisibility', 'off');
text(ax, dimXz - 0.01, traj.p0(2)/2, sprintf('z0 = %.3f m', traj.z0), ...
    'HorizontalAlignment', 'right', 'VerticalAlignment', 'middle', 'FontSize', 10, ...
    'Color', [0.20 0.20 0.20]);

dimXd = traj.p1(1) - 0.10;
plot(ax, [dimXd dimXd], [0 -traj.depth], '-', 'Color', [0.35 0.35 0.35], ...
    'LineWidth', 1.1, 'HandleVisibility', 'off');
text(ax, dimXd - 0.01, -traj.depth/2, sprintf('d = %.3f m', traj.depth), ...
    'HorizontalAlignment', 'right', 'VerticalAlignment', 'middle', 'FontSize', 10, ...
    'Color', [0.20 0.20 0.20]);

text(ax, xMin + 0.03*(xMax-xMin), 0.016, '泥面 Mud Surface', ...
    'Color', [0.10 0.40 0.95], 'FontSize', 10, 'FontWeight', 'bold');

arcR = 0.10 + 0.10 * min(1, traj.depth / 0.6);
tArc = linspace(0, thetaRadForArc(traj.thetaDeg), 40);
xArc = traj.p0(1) + arcR * cos(tArc);
yArc = traj.p0(2) + arcR * sin(tArc);
plot(ax, xArc, yArc, '-', 'Color', [0.80 0.25 0.25], 'LineWidth', 1.3, 'HandleVisibility', 'off');
tMid = tArc(round(numel(tArc)/2));
text(ax, traj.p0(1) + (arcR+0.04)*cos(tMid), traj.p0(2) + (arcR+0.04)*sin(tMid), ...
    sprintf('theta = %.1f deg', traj.thetaDeg), ...
    'Color', [0.80 0.25 0.25], 'FontSize', 10, ...
    'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle');

axis(ax, 'equal');
axis(ax, 'tight');

pad = 0.15;
xl = xlim(ax);
yl = ylim(ax);
xlim(ax, [xl(1)-pad, xl(2)+pad]);
ylim(ax, [min(yl(1)-pad, -0.15), yl(2)+pad]);

legend(ax, 'Location', 'northeast');

title(ax, sprintf('\\theta=%.1f deg, d=%.3f m, \\phi=%.1f deg', ...
    traj.thetaDeg, traj.depth, traj.phiDeg));

hold(ax, 'off');

end

function tEnd = thetaRadForArc(thetaDeg)
tEnd = deg2rad(thetaDeg);
if tEnd > 0
    tEnd = -tEnd;
end
end
