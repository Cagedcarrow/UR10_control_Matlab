function realCsvFitSendStopJ(robotIp, cmdPort, decel)
if nargin < 3
    decel = 0.5;
end
realCsvFitSendScript(robotIp, cmdPort, sprintf('stopj(%.5f)', decel));
end
