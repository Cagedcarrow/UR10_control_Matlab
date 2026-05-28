function sampledPlan = rtfg_downsample_target_plan_keep_boundaries(targetPlan, maxPts)
nPts = size(targetPlan.points, 1);
if nPts <= maxPts
    sampledPlan = targetPlan;
    sampledPlan.sampleIndices = (1:nPts).';
    return;
end

mustKeep = unique([1; targetPlan.segmentTransitionIndices(:); nPts]);
candidate = unique(round(linspace(1, nPts, maxPts))).';
idx = unique([mustKeep; candidate]);

if numel(idx) > maxPts
    optional = setdiff(idx, mustKeep, 'stable');
    keepOptionalCount = max(0, maxPts - numel(mustKeep));
    if keepOptionalCount < numel(optional)
        pick = unique(round(linspace(1, numel(optional), keepOptionalCount))).';
        optional = optional(pick);
    end
    idx = unique([mustKeep; optional]);
end

sampledPlan = targetPlan;
sampledPlan.points = targetPlan.points(idx, :);
sampledPlan.tforms = targetPlan.tforms(idx);
sampledPlan.segmentNames = targetPlan.segmentNames(idx);
sampledPlan.zAxisPreview = targetPlan.zAxisPreview(idx, :);
sampledPlan.sampleIndices = idx;
end
