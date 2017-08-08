function S = simulateK50Model(T,trial,t_range,models)

if nargin < 3
    t_range = 1:size(trial.fuse.xsmooth,2);
end

if nargin < 4
    models = 4;
end

S.models = models;
S.names = T.sname(models);
S.t_range = t_range;

F = T.F(models);
X0 = xsmoothToFullState(trial,t_range(1));

n_t = t_range(end)-t_range(1)+1;
n_x = size(X0,1);
n_m = length(models);

S.idx = zeros(n_t,n_m);
S.u = zeros(2,n_t,n_m);
S.X = zeros(n_x,n_t,n_m);
S.X_dot = zeros(n_x,n_t,n_m);

for m = 1:n_m
    S.K{m} = kmeansFromTrial(T,models(m));
    
    S.X(:,1,m) = X0;
    S.u(:,1,m) = controlFromFullState(X0,trial);    
    
    [S.X_dot(:,1,m), S.idx(1,m)] = ...
        kmeansDerivative(S.X(:,1,m),S.u(:,1,m),F{m},S.K{m});

    for t = 2:n_t
        S.X(:,t,m) = S.X(:,t-1,m) + 0.001*S.X_dot(:,t-1,m);
        S.X(7:8,t,m) = mod(S.X(7:8,t,m),2*pi);

        S.u(:,t,m) = controlFromFullState(S.X(:,t-1,m),trial);
        [S.X_dot(:,t,m),S.idx(t,m)] = ...
            kmeansDerivative(S.X(:,t-1,m),S.u(:,t,m),F{m},S.K{m});
    end
end

    