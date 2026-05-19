function traj = generate_trajectory(x0, z0, thetaDeg, d, phiDeg)
% 生成三段连续轨迹：直线切入 + 圆弧装载 + 沿姿态抬升

theta = deg2rad(thetaDeg);
phi = deg2rad(phiDeg);

p0 = [x0, z0];
dir1 = [cos(theta), sin(theta)];

downComp = -dir1(2);
if downComp < 1e-6
    downComp = 1e-6;
end
lineLen = (z0 + d) / downComp;

n1 = 80;
s1 = linspace(0, lineLen, n1)';
seg1 = p0 + s1 .* dir1;
seg1(end,2) = -d;

p1 = seg1(end,:);
R = max(0.20, 0.9 * d);

leftNormal = [-sin(theta), cos(theta)];
center = p1 + R * leftNormal;

a1 = atan2(p1(2) - center(2), p1(1) - center(1));
a2 = a1 + phi;

n2 = 140;
a = linspace(a1, a2, n2)';
seg2 = [center(1) + R*cos(a), center(2) + R*sin(a)];

p2 = seg2(end,:);

dir3 = [0, 1];

liftLen = max(0.25, 1.15 * d);

n3 = 90;
s3 = linspace(0, liftLen, n3)';
seg3 = p2 + s3 .* dir3;

seg2 = seg2(1:end-1,:);
allPts = [seg1; seg2; seg3];

dx = gradient(allPts(:,1));
dy = gradient(allPts(:,2));
heading = unwrap(atan2(dy, dx));

traj.all = allPts;
traj.seg1 = seg1;
traj.seg2 = seg2;
traj.seg3 = seg3;
traj.heading = heading;
traj.p0 = p0;
traj.p1 = p1;
traj.p2 = p2;
traj.thetaDeg = thetaDeg;
traj.depth = d;
traj.phiDeg = phiDeg;
traj.x0 = x0;
traj.z0 = z0;

end
