function ikResult = solve_mud_smearing_ik(robot, cfg, startConfig, trajectory)
%SOLVE_MUD_SMEARING_IK Solve sequential IK for the wall smearing trajectory.

ik = inverseKinematics("RigidBodyTree", robot);
try
    ik.SolverParameters.MaxIterations = 1500;
    ik.SolverParameters.AllowRandomRestart = false;
catch
end

numWaypoints = size(trajectory.targetTforms, 3);
configs = zeros(numWaypoints, numel(startConfig));
metrics = repmat(emptyMetric(), 1, numWaypoints);
seedConfig = startConfig;

for waypointIndex = 1:numWaypoints
    targetTform = trajectory.targetTforms(:, :, waypointIndex);
    bestConfig = [];
    bestMetric = [];

    for weightsIndex = 1:size(cfg.ik.weightsList, 1)
        weights = cfg.ik.weightsList(weightsIndex, :);
        try
            candidateConfig = ik(cfg.links.center, targetTform, weights, seedConfig);
        catch
            continue;
        end

        candidateMetric = evaluate_mud_smearing_solution(robot, cfg, ...
            candidateConfig, targetTform, trajectory.wall);
        if isempty(bestMetric) || candidateMetric.positionError < bestMetric.positionError
            bestMetric = candidateMetric;
            bestConfig = candidateConfig;
        end

        if candidateMetric.positionError <= cfg.ik.positionTolerance
            break;
        end
    end

    if isempty(bestConfig)
        error("IK failed at waypoint %d: solver did not return a candidate.", waypointIndex);
    end

    if bestMetric.positionError > cfg.ik.positionTolerance
        error("IK position error at waypoint %d is %.4f m, above tolerance %.4f m.", ...
            waypointIndex, bestMetric.positionError, cfg.ik.positionTolerance);
    end

    if bestMetric.maxCenterAxisAngleDeg > cfg.ik.centerAxisToleranceDeg
        warning("Waypoint %d center axis max angle is %.2f deg, above %.2f deg.", ...
            waypointIndex, bestMetric.maxCenterAxisAngleDeg, cfg.ik.centerAxisToleranceDeg);
    end

    configs(waypointIndex, :) = bestConfig;
    metrics(waypointIndex) = bestMetric;
    seedConfig = bestConfig;
end

ikResult = struct();
ikResult.configs = configs;
ikResult.metrics = metrics;
end

function metric = emptyMetric()
metric = struct( ...
    "positionError", NaN, ...
    "planeDistance", NaN, ...
    "centerXAxisAngleDeg", NaN, ...
    "centerYAxisAngleDeg", NaN, ...
    "centerZAxisAngleDeg", NaN, ...
    "maxCenterAxisAngleDeg", NaN);
end
