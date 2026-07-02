function rotation = make_frame_from_z_axis(zAxis, preferredXAxis)
%MAKE_FRAME_FROM_Z_AXIS Construct a right-handed rotation from a desired Z axis.

zAxis = normalize_vector(zAxis);
xAxis = preferredXAxis(:) - dot(preferredXAxis(:), zAxis) * zAxis;
if norm(xAxis) < 1e-9
    fallbackAxis = [1; 0; 0];
    if abs(dot(fallbackAxis, zAxis)) > 0.9
        fallbackAxis = [0; 0; 1];
    end
    xAxis = fallbackAxis - dot(fallbackAxis, zAxis) * zAxis;
end
xAxis = normalize_vector(xAxis);
yAxis = normalize_vector(cross(zAxis, xAxis));
xAxis = normalize_vector(cross(yAxis, zAxis));
rotation = [xAxis, yAxis, zAxis];
end
