function plotSimPerformance(T,SP)

colors = lines(length(SP.models));

for m = 1:length(SP.models)
%     errorbar(log10(SP.sim_length),log10(SP.scores(:,m)),log10(SP.stds(:,m)),...
%         'Color',colors(m,:));
    loglog(SP.sim_length,SP.scores);
    hold on
end
grid on
xlabel('Simulation Duration (ms)')
yl = ylabel('State Estimate Score $(D_M(X-\hat{X},X))$');
set(yl,'interpreter','latex')
title('Prediction Error vs. Simulation Duration')
legend(T.sname{SP.models})