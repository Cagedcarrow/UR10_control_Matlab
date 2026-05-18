function traj = generate_trajectory(thetaDeg, d, phiDeg)
% 生成三段连续轨迹：直线切入 + 圆弧装载 + 沿姿态抬升

theta = deg2rad(thetaDeg);
phi = deg2rad(phiDeg);

p0 = [0, 0];
dir1 = [cos(theta), sin(theta)];

n1 = 80;
s1 = linspace(0, d, n1)';
seg1 = p0 + s1 .* dir1;

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

endHeading = theta + phi;
dir3 = [cos(endHeading), sin(endHeading)];
if dir3(2) < 0
    dir3 = -dir3;
end

liftLen = max(0.25, 1.15 * d);

n3 = 90;
s3 = linspace(0, liftLen, n3)';
seg3 = p2 + s3 .* dir3;

seg1 = seg1(1:end-1,:);
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

end
