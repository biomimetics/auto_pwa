function [table,tstr] = calcTableInformation(T)

stds = std(T.D.X_dot,1,2);

n_m = 5;
n_xd = length(stds);

m_stats = zeros(n_m,n_xd);
m_stats2 = zeros(n_m,n_xd);

for m = 1:n_m
    X_dot_pred = evaluateModel(T.F{m},T.D,T.idx{m});
    err = bsxfun(@rdivide,T.D.X_dot-X_dot_pred,stds);
    m_stats(m,:) = mean(abs(err),2)';
    
    rvars = T.vars{m}';
    rrows = all(~isnan(rvars),2);
    err2 = bsxfun(@rdivide,sqrt(rvars(rrows,:))',stds);
    m_stats2(m,:) = mean(err2,2)';
end

table = [stds'; m_stats2];

tstr = '';
for r = 1:n_m+1
    for c = 1:n_xd
        tstr = [tstr sprintf('\t%.3f',table(r,c))];
    end
    tstr = [tstr sprintf('\n')];
end

