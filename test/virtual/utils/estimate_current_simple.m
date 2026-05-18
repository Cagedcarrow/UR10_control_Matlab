function i_est = estimate_current_simple(q, dq, ddq, kq, kdq, kddq, b)
q = q(:);
dq = dq(:);
ddq = ddq(:);
kq = kq(:);
kdq = kdq(:);
kddq = kddq(:);
b = b(:);
i_est = b + kq .* q + kdq .* dq + kddq .* ddq;
end
