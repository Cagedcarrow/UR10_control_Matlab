function traj3d = generate_trajectory_3d(params, envGeom)
%GENERATE_TRAJECTORY_3D Lift the YOZ-plane path into 3D space.

traj2d = generate_trajectory_yoz(params, envGeom);
xPlane = params.xPlane;

traj3d = struct();
traj3d.all = liftPoints(traj2d.all, xPlane);
traj3d.seg0 = liftPoints(traj2d.seg0, xPlane);
traj3d.seg1 = liftPoints(traj2d.seg1, xPlane);
traj3d.seg2 = liftPoints(traj2d.seg2, xPlane);
traj3d.tangent3d = computeTangents(traj3d.all);

traj3d.pApproachStart = liftPoint(traj2d.pApproachStart, xPlane);
traj3d.pEntry = liftPoint(traj2d.pEntry, xPlane);
traj3d.pArcEnd = liftPoint(traj2d.pArcEnd, xPlane);
traj3d.center = liftPoint(traj2d.center, xPlane);

traj3d.xPlane = xPlane;
traj3d.distLeft = xPlane - envGeom.basin.innerXMin;
traj3d.distRight = envGeom.basin.innerXMax - xPlane;
traj3d.mudSurfaceZ = traj2d.mudSurfaceZ;
traj3d.approachLen = traj2d.approachLen;
traj3d.thetaDeg = traj2d.thetaDeg;
traj3d.depth = traj2d.depth;
traj3d.leftWallOffset = traj2d.leftWallOffset;
traj3d.radius = traj2d.radius;
end

function pts3 = liftPoints(pts2, xPlane)
pts3 = [repmat(xPlane, size(pts2, 1), 1), pts2(:, 1), pts2(:, 2)];
end

function pt3 = liftPoint(pt2, xPlane)
pt3 = [xPlane, pt2(1), pt2(2)];
end

function tangent3d = computeTangents(pts3)
n = size(pts3, 1);
tangent3d = zeros(n, 3);

if n < 2
    tangent3d(:) = [0.0, 1.0, 0.0];
    return;
end

tangent3d(1, :) = pts3(2, :) - pts3(1, :);
tangent3d(end, :) = pts3(end, :) - pts3(end - 1, :);
for i = 2:n-1
    tangent3d(i, :) = pts3(i + 1, :) - pts3(i - 1, :);
end

tangentNorm = vecnorm(tangent3d, 2, 2);
tangentNorm(tangentNorm < 1e-9) = 1.0;
tangent3d = tangent3d ./ tangentNorm;
end
