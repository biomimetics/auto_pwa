function [data, series] = extractTrialState(trial, names, t_span)

data = [];
series = {};

if nargin < 3
  t_span = 1:length(trial.interp.t);
end

if ~iscell(names)
  names = {names};
end

for n = 1:length(names)
  name = names{n};
  new_data = [];
  if any(strcmp(name, fieldnames(trial.interp.Control)))
    new_data = trial.interp.Control.(name)'; 
  elseif any(strcmp(name, fieldnames(trial.fuse.params)))
    new_data = trial.fuse.xsmooth(trial.fuse.params.(name),:);
  elseif any(strcmp(name, fieldnames(trial.metrics)))
    new_data = trial.metrics.(name);
  end
  
  if ~isempty(new_data)
    data = [data; new_data(:,t_span)];
    for i = 1:size(new_data,1)
      series{length(series)+1} = sprintf('%s_%d',name,i);
    end
  end
end