function [P,idx] = nullSegmenter(D)

T = size(D.X,2);

P.R = {};
P.r = {};

idx = zeros(1,T);