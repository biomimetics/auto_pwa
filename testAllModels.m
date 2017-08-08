function M = testAllModels(D)

M.D = D;
M.sname = {'Null', 'Average', 'Zscore1.5', 'kMeans10','kMeans50'};
segmenters = {@nullSegmenter, @avgSegmenter, @zscore1_5, @kmeansSegmenter, @kmeans50};

M.X_dot_names = D.X_dot_names;
for s = 1:length(segmenters)
  tic();
  [F,idx,counts,vars] = testModel(D,segmenters{s},s==1);
  M.time{s} = toc()
  [M.sname{s} 'done']
  M.F{s} = F;
  M.idx{s} = idx;
  M.counts{s} = counts;
  M.vars{s} = vars;
end

function [P,idx] = zscore1_5(D)
n_x = size(D.X,1);
[P,idx] = zscoreSegmenter(D,1.5*ones(n_x,1));

function [P,idx] = kmeans50(D)
[P,idx] = kmeansSegmenter(D,50);