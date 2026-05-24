function cfg = realCsvFitDefaultConfig(baseDir)
projectDir = fileparts(baseDir);

cfg = struct();
cfg.baseDir = baseDir;
cfg.projectDir = projectDir;
cfg.robotIp = '10.160.9.21';
cfg.readPort = 30003;
cfg.cmdPort = 30002;
cfg.xacroPath = fullfile(projectDir, 'assembly', 'assembly.urdf.xacro');
cfg.meshRootName = 'meshes';
cfg.samplePeriod = 0.08;
cfg.streamStaleSec = 1.0;
cfg.defaultSampleTime = 0.02;
cfg.polyOrder = 3;
cfg.previewPeriod = 0.03;
cfg.movejA = 0.03;
cfg.movejV = 0.03;
cfg.movejT = 0.0;
cfg.movejR = 0.0;
cfg.stopDecel = 0.5;
cfg.timeScale = 5.0;
cfg.servojMinT = 0.08;
cfg.servojLookahead = 0.20;
cfg.servojGain = 100;
cfg.maxScriptPoints = 1500;
cfg.startNearTolRad = 0.12;
cfg.jointLimitsRad = repmat([-2*pi, 2*pi], 6, 1);
cfg.guiView = [135, 20];
cfg.pathMaxPoints = 200;
cfg.endEffector = 'sensor_shovel_tcp';
cfg.defaultCsvPath = findDefaultCsv(projectDir);
end

function csvPath = findDefaultCsv(projectDir)
csvPath = '';
candidates = { ...
    fullfile(fileparts(projectDir),'guijidata','05_10_173354','session_data.csv'), ...
    fullfile(projectDir,'test','data','05_10_173354','session_data.csv')};
for i = 1:numel(candidates)
    if isfile(candidates{i})
        csvPath = candidates{i};
        return;
    end
end
end
