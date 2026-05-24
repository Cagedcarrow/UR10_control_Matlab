function realCsvFitSendMoveJ(robotIp, cmdPort, q, a, v, t, r)
script = sprintf('movej([%.10f,%.10f,%.10f,%.10f,%.10f,%.10f],a=%.5f,v=%.5f,t=%.5f,r=%.5f)', ...
    q(1),q(2),q(3),q(4),q(5),q(6),a,v,t,r);
realCsvFitSendScript(robotIp, cmdPort, script);
end
