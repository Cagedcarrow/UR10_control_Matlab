function rotation = make_frame_from_x_y_axes(xAxis, yAxis)
%MAKE_FRAME_FROM_X_Y_AXES Construct a right-handed rotation from desired X/Y axes.

xAxis = normalize_vector(xAxis);
yAxis = yAxis(:) - dot(yAxis(:), xAxis) * xAxis;
if norm(yAxis) < 1e-9
    error("Desired Y axis is parallel to desired X axis.");
end
yAxis = normalize_vector(yAxis);
zAxis = normalize_vector(cross(xAxis, yAxis));
yAxis = normalize_vector(cross(zAxis, xAxis));
rotation = [xAxis, yAxis, zAxis];
end
