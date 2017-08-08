function stats = collectStats(data)

stats.mean = mean(data,2);
stats.std = std(data,0,2);
for i = 1:size(data,1)
  stats.quantiles(i,:) = quantile(data(i,:),[0.1 0.25 0.5 0.75 0.9]);
  [counts, centers] = hist(data(i,:),500);
  stats.counts(i,:) = counts;
  stats.centers(i,:) = centers;
end