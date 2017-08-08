function plotTrialData(trial, names, t_range)

if nargin < 3
  t_range = 1:length(trial.interp.t);
end

[data, series] = extractTrialState(trial, names, t_range);

if ~isempty(data)
  plot(trial.interp.t(t_range), data)
  legend(series)
  axis tight
end