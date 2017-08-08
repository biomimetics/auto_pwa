function idx = modelRegion(F,X)

idx = zeros(1,size(X,2));

for r = 1:length(F.P.R)
  inside = all(bsxfun(@le,F.P.R{r} * X,F.P.r{r}),1);
  idx(inside) = r;
end