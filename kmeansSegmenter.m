function [P,idx] = kmeansSegmenter(D,k)

if nargin < 2
  k = 10;
end

opts = statset('MaxIter',500);

% normX = D.X';
% mins = min(normX,[],1);
% maxs = max(normX,[],1);
% 
% normX = bsxfun(@rdivide,bsxfun(@minus,normX,mins),maxs-mins);

normX = zscore(D.X',1);

idx = kmeans(normX,k,'Start','cluster','Options',opts)';

P.R = cell(1,k);
P.r = cell(1,k);