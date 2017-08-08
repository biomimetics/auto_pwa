function [indices, means] = plotKMeans(data, k, plotType, mod_period)

if nargin < 3
  plotType = 'none';
end

if nargin <4
  mod_period = 0;
end

[indices,means] = kmeans(mod(data,mod_period), k);

if strcmp(plotType, 'point')
  figure();
  hold on
  colors = hsv(k);
  for i = 1:k
    plot(data(indices==i,1),data(indices==i,2), 'Color', colors(i,:),...
      'Marker', '.', 'LineStyle', 'none')
  end
  plot(means(:,1), means(:,2), 'Color', 'k', 'Marker', '*',...
      'MarkerSize',10, 'LineStyle', 'none')
  axis tight
elseif strcmp(plotType, 'adjacency')
  figure();
  hold on;
  counts = countAdjacencies(indices,k);
  
  for i = 1:k
    counts(i,:) = counts(i,:)/max(counts(i,:));
    for j = 1:k
      if counts(i,j) > 0
        x = means(i,1);
        y = means(i,2);
        u = means(j,1) - x;
        v = means(j,2) - y;
        if mod_period
          if abs(u + mod_period) < abs(u)
            u = u + mod_period;
          end
          if abs(u - mod_period) < abs(u)
            u = u - mod_period;
          end
          if abs(v + mod_period) < abs(v)
            v = v + mod_period;
          end
          if abs(v - mod_period) < abs(v)
            v = v - mod_period;
          end
        end
        quiver(x,y,u,v, 'Color',[counts(i,j) 0 0],'LineWidth',2)
      end
    end
  end
  plot(means(:,1), means(:,2), 'Color', 'b', 'Marker', 'o',...
      'MarkerSize',15, 'LineStyle', 'none')
end