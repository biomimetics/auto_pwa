function plotPredErrVsDist(R)

n_t = size(R.xr_dist{1},2);
n_model = length(R.idx);

n_bin = 20;
x_step = n_t/n_bin;

colors = lines(n_model);

bins = zeros(n_bin,n_model);
means = zeros(n_bin,n_model);
up_q = zeros(n_bin,n_model);
low_q = zeros(n_bin,n_model);

markers = {'.' 'x' '+' 'o' '*'};
sizes = [20 10 10 10 10];
for m = 1:n_model
    [d_sort, d_idx] = sort(R.xr_dist{m});
    e_sort = R.pred.dist{m}(:,d_idx);
    for xi = 1:n_bin
        d_idx = floor((xi-1)*x_step)+1:floor(xi*x_step);
        bins(xi,m) = mean(d_sort(d_idx));
        means(xi,m) = mean(e_sort(d_idx));
        quant = quantile(e_sort(d_idx),[0.25 0.75]);
        low_q(xi,m) = quant(1);
        up_q(xi,m) = quant(2);
    end
%     errorbar(bins(:,m),means(:,m),low_q(:,m),up_q(:,m),'Color',colors(m,:))
    plot(bins(:,m),means(:,m),sprintf('%s-',markers{m}),...
        'Color',colors(m,:),'LineWidth',2,'MarkerSize',sizes(m))
    hold on
    
end

axis equal
%log scale
% axis([0 0.8 0 0.5])
axis([0 6 0 4])
grid on
legend('Null', 'Avg', 'Z-1.5', 'K-10', 'K-50',...
    'Location','SouthEast')
title('Model Approach Input/Output Performance')
xlabel('Input Score D_M(K_i-X,X)')
yl = ylabel('Output Score $D_M(\dot{X}-\hat{\dot{X}},\dot{X})$');
set(yl,'interpreter','latex')

% for m = 1:n_model
%     plot(bins(:,m),low_q(:,m),':','Color',colors(m,:))
%     plot(bins(:,m),up_q(:,m),':','Color',colors(m,:))
% end

% serr = sum(zerr{1},1);
% colors = hsv(10);

%msize = 4;
% msize = 10;
% 
% figure(1)
% for r = 1:10
%   plot(r_dist(r_idx==r),serr(r_idx==r),...
%     'Color', colors(r,:),'MarkerSize',msize,...
%     'Marker', '.', 'LineStyle', 'none')
%     hold on
% end
% 
% R.serr = serr;
% 
% n = 100;
% x_max = 10;
% y_max = 20;
% xi = linspace(0,x_max,n);
% yi = linspace(0,y_max,n);
% 
% R.xi = xi;
% R.yi = yi;
% R.Z = zeros(n,n,10);
% 
% figure(2)
% 
% for r = 1:10
%     xr = interp1(xi,1:n,r_dist(r_idx==r),'nearest',NaN);
%     yr = interp1(yi,1:n,serr(r_idx==r),'nearest',NaN);
% 
%     for i = 1:length(xr)
%         if ~isnan(xr(i)) &&  ~isnan(yr(i))
%             R.Z(xr(i),yr(i),r) = R.Z(xr(i),yr(i),r) + 1;
%         end
%     end
%     
%     mid = max(max(R.Z(:,:,r)))/2;
%     contour(xi,yi,R.Z(:,:,r),[mid mid],'Color',colors(r,:),'LineWidth',3);
%     hold on
% end