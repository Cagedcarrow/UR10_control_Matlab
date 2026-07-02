function trajectory = build_wall_contact_trajectory(robot, startConfig, cfg)
%BUILD_WALL_CONTACT_TRAJECTORY Create vertical contact targets on the wall.

wallTform = getTransform(robot, startConfig, cfg.links.wall);
blockTform = getTransform(robot, startConfig, cfg.links.block);

wallOrigin = wallTform(1:3, 4);
wallXAxis = normalize_vector(wallTform(1:3, 1));
wallYAxis = normalize_vector(wallTform(1:3, 2));
wallZAxis = normalize_vector(wallTform(1:3, 3));
blockOrigin = blockTform(1:3, 4);
blockZAxis = normalize_vector(blockTform(1:3, 3));

contactNormal = choose_wall_normal_toward_robot(robot, startConfig, cfg, wallOrigin, wallYAxis);
planePoint = wallOrigin;
sideName = "centerline";

blockTopCenter = blockOrigin + blockZAxis * cfg.block.size(3) / 2;
startWorldZ = blockTopCenter(3) + cfg.trajectory.startClearanceAboveBlock;
endPoint = wallOrigin + wallZAxis * (cfg.wall.size(3) / 2 + cfg.trajectory.endDetachMargin);
endWorldZ = endPoint(3);

if abs(wallZAxis(3)) < 1e-9
    error("block_basin_back_wall local Z axis is not usable for a vertical trajectory.");
end

startOffset = (startWorldZ - wallOrigin(3)) / wallZAxis(3);
endOffset = (endWorldZ - wallOrigin(3)) / wallZAxis(3);
offsets = offsets_through_zero(startOffset, endOffset, cfg.trajectory.numWaypoints);
desiredRotation = make_frame_from_x_y_axes( ...
    cfg.trajectory.desiredCenterXAxis, ...
    cfg.trajectory.desiredCenterYAxis);

targetTforms = repmat(eye(4), 1, 1, numel(offsets));
targetPositions = zeros(numel(offsets), 3);
heights = zeros(1, numel(offsets));
for idx = 1:numel(offsets)
    targetPosition = wallOrigin + wallZAxis * offsets(idx);
    targetTforms(:, :, idx) = trvecrotm_to_tform(targetPosition, desiredRotation);
    targetPositions(idx, :) = targetPosition.';
    heights(idx) = targetPosition(3);
end

wall = struct();
wall.tform = wallTform;
wall.origin = wallOrigin;
wall.normal = contactNormal;
wall.planePoint = planePoint;
wall.sideName = sideName;
wall.vertices = wall_rectangle_vertices(planePoint, wallXAxis, wallZAxis, ...
    cfg.wall.size(1), cfg.wall.size(3));

trajectory = struct();
trajectory.targetTforms = targetTforms;
trajectory.targetPositions = targetPositions;
trajectory.heights = heights;
trajectory.startWorldZ = startWorldZ;
trajectory.endWorldZ = endWorldZ;
trajectory.wall = wall;
end

function normal = choose_wall_normal_toward_robot(robot, startConfig, cfg, wallOrigin, wallYAxis)
centerTform = getTransform(robot, startConfig, cfg.links.center);
centerPosition = centerTform(1:3, 4);
if dot(centerPosition - wallOrigin, wallYAxis) >= 0
    normal = wallYAxis;
else
    normal = -wallYAxis;
end
normal = normalize_vector(normal);
end

function offsets = offsets_through_zero(startOffset, endOffset, numWaypoints)
if numWaypoints < 2
    offsets = 0;
    return;
end

if min(startOffset, endOffset) <= 0 && max(startOffset, endOffset) >= 0
    ratioToZero = abs(startOffset) / max(abs(endOffset - startOffset), eps);
    numBefore = max(2, min(numWaypoints - 1, round(ratioToZero * (numWaypoints - 1)) + 1));
    before = linspace(startOffset, 0, numBefore);
    after = linspace(0, endOffset, numWaypoints - numBefore + 1);
    offsets = [before, after(2:end)];
else
    offsets = linspace(startOffset, endOffset, numWaypoints);
end
end

function tform = trvecrotm_to_tform(position, rotation)
tform = eye(4);
tform(1:3, 1:3) = rotation;
tform(1:3, 4) = position;
end

function vertices = wall_rectangle_vertices(center, xAxis, zAxis, width, height)
vertices = [
    (center - xAxis * width / 2 - zAxis * height / 2).'
    (center + xAxis * width / 2 - zAxis * height / 2).'
    (center + xAxis * width / 2 + zAxis * height / 2).'
    (center - xAxis * width / 2 + zAxis * height / 2).'
    ];
end
