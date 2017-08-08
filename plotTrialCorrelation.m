function [coords, counts, means, stds] = plotTrialCorrelation(...
  trial, d_vars, i_vars, outlier_thresh, t_span, n_bins)

if nargin < 2
  t_span = 1:size(trial.fuse.xsmooth,2);
end

if nargin < 3
  n_bins = 100;
end

