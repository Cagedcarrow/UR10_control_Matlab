function traj2d = generate_trajectory_yoz(params, envGeom)
%GENERATE_TRAJECTORY_YOZ Build the scoop path directly in the YOZ plane.

basin = envGeom.basin;
approachLen = envGeom.motion.approachLen;

yEntry = basin.innerYMin + params.leftWallOffset;
zMud = basin.mudSurfaceZ;
theta = deg2rad(params.thetaDeg);
radius = params.depth;

dirEntry = [cos(theta), sin(theta)];
pEntry = [yEntry, zMud];
pApproachStart = pEntry - approachLen * dirEntry;

a1 = theta - pi / 2;
a2 = -pi / 2;
center = pEntry - radius * [cos(a1), sin(a1)];

nArc = 140;
a = linspace(a1, a2, nArc)';
seg1 = [center(1) + radius * cos(a), center(2) + radius * sin(a)];
pArcEnd = seg1(end, :);

liftLen = min(basin.zMotionMax - pArcEnd(2), max(0.06, 1.20 * radius));
liftLen = max(liftLen, 0.0);
nLift = 90;
zLift = linspace(pArcEnd(2), pArcEnd(2) + liftLen, nLift)';
seg2 = [repmat(pArcEnd(1), nLift, 1), zLift];

n0 = 30;
seg0 = [linspace(pApproachStart(1), pEntry(1), n0)', ...
        linspace(pApproachStart(2), pEntry(2), n0)'];

allPts = [seg0(1:end-1, :); seg1; seg2(2:end, :)];
dy = gradient(allPts(:, 1));
dz = gradient(allPts(:, 2));
heading = unwrap(atan2(dz, dy));

traj2d = struct();
traj2d.all = allPts;
traj2d.seg0 = seg0;
traj2d.seg1 = seg1;
traj2d.seg2 = seg2;
traj2d.heading = heading;
traj2d.pApproachStart = pApproachStart;
traj2d.pEntry = pEntry;
traj2d.pArcEnd = pArcEnd;
traj2d.center = center;
traj2d.radius = radius;
traj2d.thetaDeg = params.thetaDeg;
traj2d.depth = params.depth;
traj2d.leftWallOffset = params.leftWallOffset;
traj2d.mudSurfaceZ = zMud;
traj2d.approachLen = approachLen;
end
