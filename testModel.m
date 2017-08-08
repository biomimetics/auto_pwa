function [F,idx,counts,variance] = testModel(D,segmenter,is_null)

[F,idx] = buildModel(D,segmenter);

n_r = length(F.P.R) + 1;
n_xd = size(D.X_dot,1);
variance = zeros(n_xd,n_r);
counts = zeros(1,n_r);

for r = 1:n_r
  r_idx = idx == r;
  counts(r) = sum(r_idx);
  regionXu = [D.X(:,r_idx); D.u(:,r_idx)]';
  regionX_dot = D.X_dot(:,r_idx)';
  
  currValidator = @(xua, xda, xub, xdb) ...
      regionValidator(xua, xda, xub, xdb,is_null);
  if(counts(r) > 10)
      
    variance (:,r) = mean(crossval(currValidator,regionXu,regionX_dot))';
  else
    variance(:,r) = currValidator(regionXu,regionX_dot,regionXu,regionX_dot);
  end
end

function sse = regionValidator(Xu_train, X_dot_train, Xu_test, X_dot_test,...
    is_null)

if nargin < 5
    is_null = 0;
end

n_train = size(Xu_train,1);
n_test = size(Xu_test,1);
n_xu = size(Xu_train,2);
n_xd = size(X_dot_train,2);

Regressors = [Xu_train'; ones(1,n_train)];

if size(Regressors, 2) < size(Regressors,1)
  Theta = NaN*ones(n_xd, n_xu+1);
else
    if is_null
        Theta = zeros(n_xd,n_xu+1);
        Theta(:,end) = mean(X_dot_train,1);
    else
        Theta = (Regressors'\X_dot_train)';
    end
end

sse = sum((Theta*[Xu_test'; ones(1,n_test)] - X_dot_test').^2,2)/n_test;

