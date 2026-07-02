function path = interpolate_joint_path(startConfig, endConfig, numSteps)
%INTERPOLATE_JOINT_PATH Linear interpolation in joint space.

if numSteps < 2
    path = endConfig;
    return;
end

blend = linspace(0, 1, numSteps).';
path = startConfig + blend .* (endConfig - startConfig);
end
