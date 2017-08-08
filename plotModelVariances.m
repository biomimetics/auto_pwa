function plotModelVariances(M)

n_m = length(M.vars);
n_x = size(M.vars{1},1);
data = zeros(n_x,n_m);
data(:,1) = var(M.D.X_dot,0,2);
for m = 2:n_m
  validModels = all(~isnan(M.vars{m}),1);
  data(:,m) = 1- (mean(M.vars{m}(:,validModels),2)./data(:,1));
end

figure
bar(data(:,2:end))
axis([0 9 0 1])
title('Percent Variance vs. Null Model')
legend(M.sname{2:end},'Orientation','horizontal','Location','NorthOutside')
set(gca, 'XTick', 1:n_x)
set(gca,'XTickLabel', M.X_dot_names)