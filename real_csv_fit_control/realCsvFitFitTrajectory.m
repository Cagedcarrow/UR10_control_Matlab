function fit = realCsvFitFitTrajectory(data, order)
qCoef = zeros(6, order+1);
dqCoef = zeros(6, order+1);
qFit = zeros(data.n,6);
dqFit = zeros(data.n,6);
qRmse = zeros(6,1);
dqRmse = zeros(6,1);

for j = 1:6
    pQ = polyfit(data.t, data.q(:,j), order);
    pDQ = polyfit(data.t, data.dq(:,j), order);
    qh = polyval(pQ, data.t);
    dqh = polyval(pDQ, data.t);
    qCoef(j,:) = pQ;
    dqCoef(j,:) = pDQ;
    qFit(:,j) = qh;
    dqFit(:,j) = dqh;
    qRmse(j) = sqrt(mean((qh - data.q(:,j)).^2));
    dqRmse(j) = sqrt(mean((dqh - data.dq(:,j)).^2));
end

fit = struct();
fit.t = data.t;
fit.qCoef = qCoef;
fit.dqCoef = dqCoef;
fit.qFit = qFit;
fit.dqFit = dqFit;
fit.qRmse = qRmse;
fit.dqRmse = dqRmse;
fit.order = order;
end
