function plotSimReal(S, trial, state)

n_s = length(state);
n_m = length(S.models);

X = xsmoothToFullState(trial,S.t_range);


state_names = {'x' 'y' 'z' 'r' 'p' 'yw' 'al' 'ar' ...
    'xd' 'yd' 'zd' 'rd' 'pd' 'ywd' 'ald' 'ard'};

colors = lines(n_m);
line_types = {'--' '-.' ':'};

vt = mean(trial.Control.treadSpeed(:,1));

for s = 1:n_s
    subplot(n_s+1,1,s)
    
    if state(s) == 9
        offs = vt;
    else
        offs = 0;
    end
    
    plot(offs+X(state(s),:),'k','LineWidth',2)
    hold on
    
    for m = 1:n_m
        plot(offs+S.X(state(s),:,m),line_types{m},'Color',colors(:,m))    
    end
    
    if s == 1
        legend(['Obs.' S.names],'Location','NorthOutside',...
            'Orientation','horizontal')
    end
    
    for m = 1:n_m
        Xs_rd = find(diff(S.idx(:,m)) ~= 0) + 1;
        plot(Xs_rd, offs+S.X(state(s), Xs_rd, m),'+','Color',colors(:,m))
    end
    
    ylabel(state_names{state(s)})
end
    
subplot(n_s+1,1,n_s+1)

k10i = find(S.models == 4);

if ~isempty(k10i)
    X_r = kmeansRegion(X,S.K{k10i});
    plot(X_r,'k','LineWidth',2)
    hold on
    plot(S.idx(:,k10i),'rx')

    legend('Obs.','K-10','Location','SouthOutside','Orientation','horizontal')
    ylabel('Submodel Index')
    xlabel('Simulation Time (ms)')
end
% 
% ls = {};
% for i = 1:length(state)
%     ls{i} = state_names{state(i)};
% end
% legend(ls)
% 
% title(sprintf('%d',S.t_range(1)))