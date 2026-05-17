function q = validate_joint_vector(q, limits)
q = double(q(:))';
if numel(q) ~= 6
    error('关节角维度必须为6。');
end
if any(~isfinite(q))
    error('关节角包含非有限值。');
end
if nargin >= 2 && ~isempty(limits)
    if any(q < limits(:,1)' | q > limits(:,2)')
        error('关节角超限。');
    end
end
end
