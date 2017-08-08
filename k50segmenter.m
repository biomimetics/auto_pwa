function [P,idx] = k50segmenter(D)

[P,idx] = kmeansSegmenter(D,50);