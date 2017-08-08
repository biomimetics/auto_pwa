function [P,idx] = binSegmenter(D,n_bins,use_quantiles)

if nargin < 2
  n_bins = 5;
end

if nargin < 3
  use_quantiles = 0;
end