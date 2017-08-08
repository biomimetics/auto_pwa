function [coords, counts, means, stds] = binCorrelation(error, state, n_bins, binType)
% NOTE: this expects input vector observations to be in rows, but switches
% convention to be columns for easier output. input should be (axis, obs)

error = error';
state = state';

% coords(n_bins, n_state) = edges of bins
% counts(n_bins ^ n_state) = count(b3,b2,b1) =
%   sum(bin(state(3))==i3 & bin(state(2,:))==b2 & bin(state(1,:))==b1))
% means(n_bins^n_state, n_error) = means of error_n in above bins
% std(n_bins^n_state, n_error) = stds = error_n in above bins

if nargin < 4
  binType = 'uniform';
end

n_error = size(error,2);
n_state = size(state,2);
n_sample = size(state,1);

coords = zeros(n_bins, n_state);

if strcmp(binType, 'uniform')
  if n_state == 1
    counts = zeros(n_bins,1);
  else
    counts = zeros(n_bins * ones(1,n_state));
  end
  means = squeeze(zeros([size(counts) n_error]));
  stds = squeeze(zeros([size(counts) n_error]));
  
  state_indices = zeros(n_sample, n_state);
  
  for s = 1:n_state
    coords(:,s) = linspace(min(state(:,s)),max(state(:,s)),n_bins);
    %coords(end,s) = 1.01*coords(end,s);
    [count, idx] = histc(state(:,s),coords(:,s));
    state_indices(:,s) = idx;
  end
  
  bin_idx = ones(1,n_state);
  bin_match = zeros(n_sample, n_state);
  
  for i = 1:(n_bins^n_state)
    for s = 1:n_state
      bin_match(:,s) = state_indices(:,s) == bin_idx(s);
    end

    error_idx = all(bin_match,2);
    counts(i) = sum(error_idx);
    
    points = error(error_idx,:);
    bin_means = mean(points,1);
    bin_stds = std(points,0,1);
    for e = 1:n_error
      out_idx = i+(n_bins^n_state * (e-1));
      means(out_idx) = bin_means(e);
      stds(out_idx) = bin_stds(e);
    end
    
    bin_idx(1) = bin_idx(1) + 1;
    for j = 1:(n_state-1)
      if bin_idx(j) > n_bins
        bin_idx(j) = 1;
        bin_idx(j+1) = bin_idx(j+1) + 1;
      end
    end
  end
end