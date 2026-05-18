function ensure_virtual_from_workspace_signals(sampleTime, nSamples)
% 为 From Workspace 块准备占位变量，避免 base workspace 缺变量报错

if nargin < 1 || isempty(sampleTime)
    sampleTime = 0.02;
end
if nargin < 2 || isempty(nSamples)
    nSamples = 2;
end

t = (0:nSamples-1)' * sampleTime;
z = zeros(nSamples, 6);

assignin('base','q_cmd_ts', timeseries(z, t));
assignin('base','q_ref_ts', timeseries(z, t));
assignin('base','dq_ref_ts', timeseries(z, t));
assignin('base','tau_csv_ref_ts', timeseries(z, t));
assignin('base','tau_from_i_ref_ts', timeseries(z, t));
end
