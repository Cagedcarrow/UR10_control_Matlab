function [T, pos, rpy] = fk_tcp_pose(robot, q)
q = validate_joint_vector(q, []);
T = getTransform(robot, q, 'sensor_shovel_tcp');
pos = tform2trvec(T);
rpy = tform2eul(T, 'XYZ');
end
