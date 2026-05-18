function plot_trajectory(ax, traj)
% 绘制地面、三段轨迹和姿态方向箭头

cla(ax);
hold(ax, 'on');

xAll = traj.all(:,1);
yAll = traj.all(:,2);

xMin = min(xAll) - 0.35;
xMax = max(xAll) + 0.35;
plot(ax, [xMin xMax], [0 0], 'k-', 'LineWidth', 1.3, 'DisplayName', '地面线');

plot(ax, traj.seg1(:,1), traj.seg1(:,2), '-', 'Color', [0.85 0.25 0.25], ...
    'LineWidth', 2.4, 'DisplayName', '阶段1: 直线切入');
plot(ax, traj.seg2(:,1), traj.seg2(:,2), '-', 'Color', [0.15 0.45 0.90], ...
    'LineWidth', 2.4, 'DisplayName', '阶段2: 圆弧装载');
plot(ax, traj.seg3(:,1), traj.seg3(:,2), '-', 'Color', [0.12 0.62 0.32], ...
    'LineWidth', 2.4, 'DisplayName', '阶段3: 抬升退出');

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
