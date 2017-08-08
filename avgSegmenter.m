function [P,idx] = avgSegmenter(D)

n_x = size(D.X,1);
T = size(D.X,2);

P.R{1} = zeros(1,n_x);
P.r{1} = 0;

idx = ones(1,T);