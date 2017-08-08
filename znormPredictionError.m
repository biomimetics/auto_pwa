function pred = znormPredictionError(T,m_idx)

if nargin < 2
    m_idx = 1:length(T.F);
end

i = 1;

Exd = cov(T.D.X_dot',1);

for m = m_idx
    X_dot_pred = evaluateModel(T.F{m},T.D,T.idx{m});
    err = T.D.X_dot-X_dot_pred; 
    dist{m} = sqrt(sum(err.* (Exd^-1 * err),1));
    i = i+1;
end

%pred.Exd = Exd;
%pred.err = err;
pred.dist = dist;