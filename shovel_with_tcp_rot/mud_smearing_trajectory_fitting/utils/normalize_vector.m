function unitVector = normalize_vector(vector)
%NORMALIZE_VECTOR Return a column unit vector.

unitVector = vector(:);
vectorNorm = norm(unitVector);
if vectorNorm < 1e-12
    error("Cannot normalize a near-zero vector.");
end
unitVector = unitVector / vectorNorm;
end
