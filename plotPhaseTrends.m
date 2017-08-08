function [speeds, counts, centers] = plotPhaseTrends(gains,statData)

n_bins = 100;
n_speeds = size(gains,1);

speeds = zeros(1,n_speeds);
counts = zeros(n_bins,n_speeds+1);
centers = zeros(n_bins,n_speeds+1);

for s = 1:n_speeds
  speeds(s) = gains(s,1);
  legend_str{s} = num2str(speeds(s));
  [count,center] = hist(statData(2,statData(1,:) == speeds(s)),100);
  counts(:,s) = count/sum(count);
  centers(:,s) = center;
end

[count,center] = hist(statData(2,:),100);
counts(:,n_speeds+1) = count/sum(count);
centers(:,n_speeds+1) = center;
legend_str{n_speeds+1} = 'Avg';
% 
% figure
% cmap = colormap(jet(n_speeds));
% hold on
% for s = 1:n_speeds
%   plot(centers(:,s),counts(:,s),'Color',cmap(s,:))
% end
% 
% plot(centers(:,n_speeds+1), counts(:,n_speeds+1),'Color',[0 0 0], 'LineWidth',3)
% axis([-pi pi 0 0.02])
% legend(legend_str)
% title('Phase Distribution vs. Forward Velocity')
% xlabel('Phase (rad)')
% ylabel('P(\theta_R - \theta_L = Phase)')
% 
% figure
% % NOTE this doesn't actually plot the end of counts, which is the mean of
% % the rows
% pcolor([speeds 0.55],centers(:,end),counts)
% title('Phase Distribution vs. Forward Velocity')
% xlabel('Velocity (m/s)')
% ylabel('Phase (rad)')
% shading flat

cdata = counts(:,1:(end-1));
[~,h] = contourf(speeds,centers(:,end),cdata/max(max(cdata)));
set(h,'LineWidth',2)
colormap gray
cb = colorbar('peer',gca);
set(get(cb,'ylabel'),'String','Normalized Likelihood');

title('Phase Distribution vs. Forward Velocity')
xlabel('Velocity (m/s)')
ylabel('Phase (rad)')
%shading flat