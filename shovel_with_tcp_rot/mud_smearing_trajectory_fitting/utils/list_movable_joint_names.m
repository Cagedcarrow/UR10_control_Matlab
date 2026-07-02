function jointNames = list_movable_joint_names(robot)
%LIST_MOVABLE_JOINT_NAMES Return non-fixed joint names in config-vector order.

jointNames = strings(0, 1);
for bodyIndex = 1:robot.NumBodies
    joint = robot.Bodies{bodyIndex}.Joint;
    if strcmp(joint.Type, "fixed")
        continue;
    end
    jointNames(end + 1, 1) = string(joint.Name); %#ok<AGROW>
end
end
