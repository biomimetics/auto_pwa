function plotStats(stats)

figure

n_plots = size(stats.mean,1);

for i = 1:n_plots
  subplot(n_plots,1,i)
  hold on
  y_max = max(stats.counts(i,:)/sum(stats.counts(i,:)));
  plot(stats.centers(i,:), stats.counts(i,:)/sum(stats.counts(i,:)))

  plot([stats.mean(i) stats.mean(i)],[0 y_max], '--')
  %plot([stats.std stats.std],[0 y_max], '--')

  plot([stats.quantiles(i,:)' stats.quantiles(i,:)']',repmat([0 y_max],length(stats.quantiles(i,:)),1)', '-.')
end