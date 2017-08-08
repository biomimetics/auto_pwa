function X_dot_pred = evaluateModel(F,D,idx)

if nargin < 3
  idx = modelRegion(F,D.X);
end

n_xd = size(F.A{1},1);
n_t = size(D.X,2);
X_dot_pred = zeros(n_xd,n_t);

for r = 1:length(F.A)
  region_idx = idx == r;
  X_dot_pred(:,region_idx) = [F.A{r} F.B{r} F.f{r}] * ...
    [D.X(:,region_idx); D.u(:,region_idx); ones(1,sum(region_idx))];
end