function realCsvFitSendScript(robotIp, cmdPort, script)
client = tcpclient(robotIp, cmdPort, 'Timeout', 2);
line = uint8([script newline]);
write(client, line, 'uint8');
end
