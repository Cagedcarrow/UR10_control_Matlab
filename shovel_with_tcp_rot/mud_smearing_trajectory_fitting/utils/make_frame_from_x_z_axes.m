function rotation = make_frame_from_x_z_axes(xAxis, zAxis)
%MAKE_FRAME_FROM_X_Z_AXES Construct a right-handed frame from desired X/Z axes.

xAxis = normalize_vector(xAxis);
zAxis = normalize_vector(zAxis);
xAxis = xAxis - dot(xAxis, zAxis) * zAxis;
if norm(xAxis) < 1e-9
    error("Desired X axis is parallel to desired Z axis.");
end
xAxis = normalize_vector(xAxis);
yAxis = normalize_vector(cross(zAxis, xAxis));
xAxis = normalize_vector(cross(yAxis, zAxis));
rotation = [xAxis, yAxis, zAxis];
end
