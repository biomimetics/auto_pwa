function plotPhaseHeatmap(T,trial)

X = xsmoothToFullState(trial);

al = X(7,:);
ar = X(8,:);

m_idx = 5;

n_bin = 75;
bins = linspace(0,2*pi,n_bin);

al_r = interp1(bins,1:n_bin,al,'nearest',NaN);
ar_r = interp1(bins,1:n_bin,ar,'nearest',NaN);

% trans_idx = find(diff(T.idx{m_idx}) ~= 0)+1;
% al_i = al_r(trans_idx);
% ar_i = ar_r(trans_idx);

counts = zeros(n_bin);
totals = zeros(n_bin);
for i = 1:length(T.idx{m_idx})
    if ~isnan(al_r(i)) && ~isnan(ar_r(i))
        totals(al_r(i),ar_r(i)) = totals(al_r(i),ar_r(i)) + 1;
        if i > 1 && T.idx{m_idx}(i-1) ~= T.idx{m_idx}(i)
            counts(al_r(i),ar_r(i)) = counts(al_r(i),ar_r(i)) + 1;
        end
    end
end

% Transpose is necessary since pcolor does row-major indexing
% cdata = counts'/max(max(counts));
cdata = (counts./totals)';
cmax = max(max(cdata));

usamp = totals' < 0;


cdata(usamp) = -cmax/63;
% cdata(~usamp) = cmax;
didx = 3:n_bin;

pcolor(bins(didx),bins(didx),cdata(didx,didx))

cm = colormap('gray');
cm = cm(64:-1:1,:);
% cm(1,:) = [1 0 0];
colormap(cm);

shading flat
axis equal
axis tight

title('K-50 Phase Space Transition Heatmap')
xlabel('\alpha_l (rad)')
ylabel('\alpha_r (rad)')


cb = colorbar('peer',gca);
set(get(cb,'ylabel'),'String','Transition Likelihood');

% % Mask for statistically insignificant bins
% hold on
% mask = zeros(size(cdata));
% mask(usamp) = 1;
% % mask(~usamp) = NaN;
% pcolor(bins(didx),bins(didx),mask(didx,didx))
% shading flat