function [F,idx] = buildModel(D,segmenter)

[P,idx] = segmenter(D);

F.P = P;
n_x = size(D.X,1);
n_xd = size(D.X_dot,1);
n_u = size(D.u,1);

n_r = length(P.R);

for r = 1:n_r
  r_idx = idx == r;
  Regressors = [D.X(:,r_idx); D.u(:,r_idx); ones(1,sum(r_idx))];
  if size(Regressors, 2) < (n_x + n_u + 1)
    Theta = NaN*ones(n_xd, n_x + n_u + 1);
  else
    Theta = (Regressors'\D.X_dot(:,r_idx)')';
  end
  F.A{r} = Theta(:,1:n_x);
  F.B{r} = Theta(:,(n_x+1):(end-1));
  F.f{r} = Theta(:,end);
end

% Build null model for all unclustered points
null_idx = idx==0;
idx(null_idx) = n_r+1;

F.A{n_r+1} = zeros(n_xd,n_x);
F.B{n_r+1} = zeros(n_xd,n_u);
F.f{n_r+1} = mean(D.X_dot(:,null_idx),2);
