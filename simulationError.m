function err = simulationError(T,trial)

X = xsmoothToFullState(trial);

steps = 1:100:size(trial.fuse.xsmooth,2);

err = zeros(size(X,1),length(steps));

i = 1;
for t = steps(1:(end-1))
    t_range = (1:100)+t;
    S = simulateK50Model(T,trial,t_range,1);
    err(:,i) = sum((X(:,t_range) - S.X).^2,2)./var(X(:,t_range),0,2);
    i = i + 1;
end