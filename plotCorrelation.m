function [coords, counts, means, stds] = ...
    plotCorrelation(error, data, n_bins, e_labels, d_labels,f_idx)

if nargin < 3
  n_bins = 100;
end

n_error = size(error,1);
n_data = size(data,1);

if nargin < 4
  for e = 1:n_error
    e_labels{e} = ['var ' num2str(e)];
  end
end

if nargin < 5
  for d = 1:n_data
    d_labels{d} = ['data ' num2str(d)];
  end
end

if nargin < 6
    f_idx = NaN;
end

[coords, counts, means, stds] = binCorrelation(error, data, n_bins);

if  n_data == 1
  figure
  subplot(2,1,1)
  bar(coords(1:(end-1),:), counts(1:(end-1),:), 'histc')
  xlabel(d_labels{1});
  axis tight
  
  subplot(2,1,2)
  hold on

  cmap = lines(n_error);
  for e = 1:n_error
    handles(e) = plot(coords(1:(end-1),:), means(1:(end-1),e), 'Color', cmap(e,:));
    plot(coords(1:(end-1)), [means(1:(end-1),e) + stds(1:(end-1),e) means(1:(end-1),e)-stds(1:(end-1),e)], ...
      'Color', cmap(e,:), 'Marker', '.', 'LineStyle', 'none');
  end
  xlabel(d_labels{1});
  legend(handles, e_labels);
  axis tight

elseif n_data == 2
  if isnan(f_idx)
      figure
      % Need to transpose for pcolor since it indexes x as the rows
      pcolor(coords(:,1), coords(:,2), counts)
      title('Measurement Count')
      xlabel(d_labels{1});
      ylabel(d_labels{2});
      shading flat

      figure
  end
  for e = 1:n_error
      if(isnan(f_idx))
        subplot(2,n_error,2*(e-1)+1)
        pcolor(coords(:,1), coords(:,2), stds(:,:,e))
        title(['Std Dev ' e_labels{e}])
        xlabel(d_labels{1});
        ylabel(d_labels{2});
        shading flat

        subplot(2,n_error,2*(e-1)+2)
      else
          figure(f_idx)
      end
        title(['Mean ' e_labels{e}])
        pcolor(coords(:,1), coords(:,2), means(:,:,e))
        xlabel(d_labels{1});
        ylabel(d_labels{2});
        shading flat    
  end
end
