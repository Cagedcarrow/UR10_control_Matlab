function metric = evaluate_mud_smearing_solution(robot, cfg, config, targetTform, wall)
%EVALUATE_MUD_SMEARING_SOLUTION Compute position, plane, and normal errors.

actualTform = getTransform(robot, config, cfg.links.center);
actualPosition = actualTform(1:3, 4);
targetPosition = targetTform(1:3, 4);
centerXAxis = normalize_vector(actualTform(1:3, 1));
centerYAxis = normalize_vector(actualTform(1:3, 2));
centerZAxis = normalize_vector(actualTform(1:3, 3));
desiredCenterXAxis = normalize_vector(cfg.trajectory.desiredCenterXAxis);
desiredCenterYAxis = normalize_vector(cfg.trajectory.desiredCenterYAxis);
desiredCenterZAxis = normalize_vector(cfg.trajectory.desiredCenterZAxis);

metric = struct();
metric.positionError = norm(actualPosition - targetPosition);
metric.planeDistance = abs(dot(actualPosition - wall.planePoint, wall.normal));
metric.centerXAxisAngleDeg = acosd(clamp(dot(centerXAxis, desiredCenterXAxis), -1, 1));
metric.centerYAxisAngleDeg = acosd(clamp(dot(centerYAxis, desiredCenterYAxis), -1, 1));
metric.centerZAxisAngleDeg = acosd(clamp(dot(centerZAxis, desiredCenterZAxis), -1, 1));
metric.maxCenterAxisAngleDeg = max([ ...
    metric.centerXAxisAngleDeg, ...
    metric.centerYAxisAngleDeg, ...
    metric.centerZAxisAngleDeg]);
end
