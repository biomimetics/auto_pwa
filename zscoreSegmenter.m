function [P,idx] = zscoreSegmenter(D,thresholds)

n_x = size(D.X,1);

if nargin < 2
  thresholds = ones(n_x,1);
end

means = mean(D.X,2);
stds = thresholds.*std(D.X,0,2);

P.R{1} = [eye(n_x); -eye(n_x)];
P.r{1} = [means + stds; -means + stds];

F.P = P;
idx = modelRegion(F,D.X);

% hack to make it learn a model for unclustered points
P.R{2} = zeros(size(P.R{1}));
P.r{2} = zeros(size(P.r{1}));
idx(idx==0) = 2;